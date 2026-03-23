#!/usr/bin/env python3

import math
from dataclasses import dataclass

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


def _quat_to_yaw(q: Quaternion) -> float:
    # yaw (Z) from quaternion
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _wrap_to_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


@dataclass
class _Pose2D:
    x: float
    y: float
    yaw: float
    stamp: Time


class TfOdomPublisher(Node):
    """Publish nav_msgs/Odometry by reading TF(odom->base_link).

    Why this exists:
    - When rf2o is disabled, we still need a /odom topic for Nav2 + monitors.
    - Cartographer can publish TF(odom->base_link) when provide_odom_frame=true.
    - This node converts that TF into an Odometry message.

    Notes:
    - Twist is computed via finite difference and expressed in base_link frame.
    - If TF is missing or time jumps backwards, we publish pose and zero twist.
    """

    def __init__(self):
        super().__init__('tf_odom_publisher')

        odom_frame = self.declare_parameter('odom_frame', 'odom').value
        base_frame = self.declare_parameter('base_frame', 'base_link').value
        odom_topic = self.declare_parameter('odom_topic', '/odom').value
        publish_rate = float(self.declare_parameter('publish_rate', 20.0).value)

        self._odom_frame = odom_frame
        self._base_frame = base_frame

        self._pub = self.create_publisher(Odometry, odom_topic, 10)

        # Keep a small cache; TF in this stack is mostly "latest" lookups.
        self._tf_buffer = Buffer(cache_time=Duration(seconds=2.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._last_pose: _Pose2D | None = None

        period = 1.0 / max(publish_rate, 0.5)
        self._timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f'Publishing {odom_topic} from TF({odom_frame}->{base_frame}) @ {1.0/period:.1f} Hz'
        )

    def _on_timer(self):
        try:
            tf = self._tf_buffer.lookup_transform(
                self._odom_frame,
                self._base_frame,
                Time(),  # latest
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self.get_logger().warn(f'No TF {self._odom_frame}->{self._base_frame}: {exc}', throttle_duration_sec=2.0)
            return

        # Build pose
        x = float(tf.transform.translation.x)
        y = float(tf.transform.translation.y)
        yaw = _quat_to_yaw(tf.transform.rotation)

        # TF stamp may be 0 in some pipelines; fall back to now.
        stamp = Time.from_msg(tf.header.stamp)
        if stamp.nanoseconds == 0:
            stamp = self.get_clock().now()

        current = _Pose2D(x=x, y=y, yaw=yaw, stamp=stamp)

        odom_msg = Odometry()
        odom_msg.header.stamp = stamp.to_msg()
        odom_msg.header.frame_id = self._odom_frame
        odom_msg.child_frame_id = self._base_frame
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = float(tf.transform.translation.z)
        odom_msg.pose.pose.orientation = tf.transform.rotation

        # Default twist to 0; fill if we have a sane dt.
        if self._last_pose is not None:
            dt = (current.stamp - self._last_pose.stamp).nanoseconds / 1e9
            if 0.001 <= dt <= 1.0:
                dx = current.x - self._last_pose.x
                dy = current.y - self._last_pose.y
                dyaw = _wrap_to_pi(current.yaw - self._last_pose.yaw)

                vx_odom = dx / dt
                vy_odom = dy / dt

                # Rotate odom-frame velocity into base_link frame.
                c = math.cos(current.yaw)
                s = math.sin(current.yaw)
                vx_base = c * vx_odom + s * vy_odom
                vy_base = -s * vx_odom + c * vy_odom

                odom_msg.twist.twist.linear.x = float(vx_base)
                odom_msg.twist.twist.linear.y = float(vy_base)
                odom_msg.twist.twist.angular.z = float(dyaw / dt)

        self._last_pose = current
        self._pub.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TfOdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
