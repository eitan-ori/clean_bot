#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from tf2_ros import TransformBroadcaster
import serial
import math


class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')

        # Parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 57600)
        self.declare_parameter('wheel_radius', 0.05)  # meters (10cm diameter)
        self.declare_parameter('wheel_base', 0.21)    # meters (21cm track width)
        self.declare_parameter('ticks_per_rev', 3000)  # encoder ticks per revolution

        self.port = self.get_parameter('serial_port').value
        self.baud = self.get_parameter('baud_rate').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').value

        # Serial Connection
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            self.get_logger().info(f'Connected to Arduino on {self.port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to Arduino: {e}')
            self.ser = None

        # Publishers & Subscribers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.range_pub = self.create_publisher(Range, 'ultrasonic', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # Odometry State
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()

        self.left_ticks_prev = 0
        self.right_ticks_prev = 0
        self.first_run = True

        # Timer for reading serial
        self.create_timer(0.05, self.update_odometry)  # 20Hz

    def cmd_vel_callback(self, msg):
        if not self.ser:
            return

        # Differential Drive Kinematics
        linear = msg.linear.x
        angular = msg.angular.z

        # Calculate wheel velocities (m/s)
        v_left = linear - (angular * self.wheel_base / 2.0)
        v_right = linear + (angular * self.wheel_base / 2.0)

        # Convert to PWM or internal units (Simple protocol: "L,R\n")
        # Here we send raw velocity, Arduino should handle PID
        cmd_str = f"{v_left:.3f},{v_right:.3f}\n"
        self.ser.write(cmd_str.encode('utf-8'))

    def update_odometry(self):
        if not self.ser or not self.ser.in_waiting:
            return

        try:
            line = self.ser.readline().decode('utf-8').strip()
            # Protocol: "left_ticks,right_ticks,distance_cm"
            parts = line.split(',')
            if len(parts) != 3:
                return

            left_ticks = int(parts[0])
            right_ticks = int(parts[1])
            distance_cm = float(parts[2])

            if self.first_run:
                self.left_ticks_prev = left_ticks
                self.right_ticks_prev = right_ticks
                self.first_run = False
                return

            # Calculate delta ticks
            d_left_ticks = left_ticks - self.left_ticks_prev
            d_right_ticks = right_ticks - self.right_ticks_prev

            self.left_ticks_prev = left_ticks
            self.right_ticks_prev = right_ticks

            # Calculate distance per wheel
            d_left = (d_left_ticks / self.ticks_per_rev) * (2 * math.pi * self.wheel_radius)
            d_right = (d_right_ticks / self.ticks_per_rev) * (2 * math.pi * self.wheel_radius)

            # Calculate robot movement
            d_center = (d_left + d_right) / 2.0
            d_theta = (d_right - d_left) / self.wheel_base

            # Update pose
            self.x += d_center * math.cos(self.th)
            self.y += d_center * math.sin(self.th)
            self.th += d_theta

            # Publish Odom & TF
            self.publish_odom()

            # Publish Ultrasonic
            self.publish_ultrasonic(distance_cm)

        except ValueError:
            pass
        except Exception as e:
            self.get_logger().warn(f'Serial read error: {e}')

    def publish_odom(self):
        current_time = self.get_clock().now()

        # Quaternion from Yaw
        q = quaternion_from_euler(0, 0, self.th)

        # Publish TF: odom -> base_link
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

        # Publish Odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q

        # Velocity (optional, calculated from d_center/dt)
        # odom.twist.twist.linear.x = ...
        # odom.twist.twist.angular.z = ...

        self.odom_pub.publish(odom)

    def publish_ultrasonic(self, distance_cm):
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ultrasonic_link'
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = 0.26  # ~15 degrees
        msg.min_range = 0.02
        msg.max_range = 4.0
        msg.range = distance_cm / 100.0
        self.range_pub.publish(msg)


def quaternion_from_euler(roll, pitch, yaw):
    qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - \
        math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + \
        math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
    qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - \
        math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
    qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + \
        math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
