#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String, Bool
import math


class AutoDocker(Node):
    def __init__(self):
        super().__init__('auto_docker')

        self.sub_dock = self.create_subscription(PoseStamped, '/my_dock_pose', self.dock_callback, 10)
        self.sub_trigger = self.create_subscription(Bool, '/start_docking', self.trigger_callback, 10)
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_status = self.create_publisher(String, '/dock_status', 10)

        self.docking_active = False
        self.target_dist = 0.30
        self.last_dock_time = self.get_clock().now()
        
        # Last observed dock pose (in laser frame)
        self.last_dock_x = None
        self.last_dock_y = None
        
        # Periodic control so we can "search" even when dock_pose is not available
        self.create_timer(0.05, self.control_loop)  # 20Hz

    def trigger_callback(self, msg):
        if msg.data:
            self.docking_active = True
            self.last_dock_x = None
            self.last_dock_y = None
            self.get_logger().info("Docking Activated!")

    def dock_callback(self, msg):
        if not self.docking_active:
            return

        self.get_logger().info(f"Received dock pose: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}")
        self.last_dock_time = self.get_clock().now()

        # Dock pose is in laser_frame (robot frame roughly)
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.last_dock_x = float(x)
        self.last_dock_y = float(y)

    def control_loop(self):
        if not self.docking_active:
            return

        now = self.get_clock().now()
        dt_since = (now - self.last_dock_time).nanoseconds / 1e9

        cmd = Twist()

        # If we haven't seen the dock recently, rotate in place to find it.
        if self.last_dock_x is None or dt_since > 1.0:
            if self.last_dock_x is None:
                self.get_logger().info("Searching: No dock pose received yet.")
            else:
                self.get_logger().info(f"Searching: Last dock pose too old ({dt_since:.2f}s).")
            
            cmd.linear.x = 0.0
            cmd.angular.z = 0.2
            self.pub_status.publish(String(data="searching"))
            self.pub_cmd.publish(cmd)
            return

        x = float(self.last_dock_x)
        y = float(self.last_dock_y)

        dist = math.sqrt(x * x + y * y)
        angle = math.atan2(y, x)
        
        self.get_logger().info(f"Control: dist={dist:.2f}, angle={angle:.2f}, target={self.target_dist}")

        if dist > self.target_dist + 0.01:
            # P-Controller
            cmd.linear.x = 0.2 * (dist - self.target_dist)
            if cmd.linear.x < 0.05:
                cmd.linear.x = 0.05
            cmd.angular.z = 1.5 * angle

            # Limits
            if cmd.linear.x > 0.2:
                cmd.linear.x = 0.2
            if cmd.angular.z > 0.5:
                cmd.angular.z = 0.5
            if cmd.angular.z < -0.5:
                cmd.angular.z = -0.5

            self.pub_status.publish(String(data="approaching"))
        else:
            # Arrived
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.pub_status.publish(String(data="docked"))
            self.docking_active = False
            self.get_logger().info("Docking Complete!")

        self.pub_cmd.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = AutoDocker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
