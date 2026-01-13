#!/usr/bin/env python3
"""
Coverage Test Helper Script

This script helps test the coverage mission by:
1. Monitoring robot status in real-time
2. Sending commands easily
3. Logging useful debug info

Usage:
    python3 test_helper.py monitor    # Monitor robot status
    python3 test_helper.py clean      # Send clean command
    python3 test_helper.py stop       # Send stop command
    python3 test_helper.py status     # Get current status
"""

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math


class TestHelper(Node):
    def __init__(self, mode='monitor'):
        super().__init__('test_helper')
        self.mode = mode
        
        # Publishers
        self.cmd_pub = self.create_publisher(String, 'mission_command', 10)
        self.coverage_cmd_pub = self.create_publisher(String, 'coverage_control', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel_nav', self.cmd_vel_callback, 10)
        self.state_sub = self.create_subscription(
            String, 'coverage_state', self.state_callback, 10)
        self.mission_state_sub = self.create_subscription(
            String, 'mission_state', self.mission_state_callback, 10)
        
        # State
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.last_cmd_linear = 0.0
        self.last_cmd_angular = 0.0
        self.coverage_state = "UNKNOWN"
        self.mission_state = "UNKNOWN"
        
        if mode == 'monitor':
            self.create_timer(1.0, self.print_status)
            self.get_logger().info('📊 Monitoring robot status... (Ctrl+C to stop)')
        elif mode == 'clean':
            self.send_clean_command()
        elif mode == 'stop':
            self.send_stop_command()
        elif mode == 'status':
            self.create_timer(0.5, self.print_once_and_exit)
            
    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
        
    def cmd_vel_callback(self, msg):
        self.last_cmd_linear = msg.linear.x
        self.last_cmd_angular = msg.angular.z
        
    def state_callback(self, msg):
        self.coverage_state = msg.data
        
    def mission_state_callback(self, msg):
        self.mission_state = msg.data
        
    def print_status(self):
        # Determine what robot is doing
        if abs(self.last_cmd_linear) > 0.01 and abs(self.last_cmd_angular) < 0.05:
            action = "🚗 DRIVING"
        elif abs(self.last_cmd_angular) > 0.01:
            action = "🔄 TURNING"
        else:
            action = "⏸️ STOPPED"
            
        print(f'\r{action} | '
              f'Pos: ({self.robot_x:6.2f}, {self.robot_y:6.2f}) | '
              f'Yaw: {math.degrees(self.robot_yaw):6.1f}° | '
              f'Cmd: lin={self.last_cmd_linear:5.2f} ang={self.last_cmd_angular:5.2f} | '
              f'Coverage: {self.coverage_state:10s} | '
              f'Mission: {self.mission_state}',
              end='', flush=True)
        
    def print_once_and_exit(self):
        print(f'''
========================================
Robot Status
========================================
Position: ({self.robot_x:.3f}, {self.robot_y:.3f})
Yaw: {math.degrees(self.robot_yaw):.1f}°
Last cmd_vel: linear={self.last_cmd_linear:.3f}, angular={self.last_cmd_angular:.3f}
Coverage State: {self.coverage_state}
Mission State: {self.mission_state}
========================================
''')
        rclpy.shutdown()
        
    def send_clean_command(self):
        msg = String()
        msg.data = 'start_clean'
        self.cmd_pub.publish(msg)
        self.get_logger().info('📤 Sent: start_clean')
        
        # Also send directly to coverage
        msg2 = String()
        msg2.data = 'start'
        self.coverage_cmd_pub.publish(msg2)
        self.get_logger().info('📤 Sent to coverage_control: start')
        
        self.create_timer(1.0, lambda: rclpy.shutdown())
        
    def send_stop_command(self):
        msg = String()
        msg.data = 'stop_clean'
        self.cmd_pub.publish(msg)
        self.get_logger().info('📤 Sent: stop_clean')
        
        msg2 = String()
        msg2.data = 'stop'
        self.coverage_cmd_pub.publish(msg2)
        self.get_logger().info('📤 Sent to coverage_control: stop')
        
        self.create_timer(1.0, lambda: rclpy.shutdown())


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else 'monitor'
    
    if mode not in ['monitor', 'clean', 'stop', 'status']:
        print(f'''
Usage: python3 test_helper.py <mode>

Modes:
    monitor  - Watch robot status in real-time
    clean    - Send start_clean command
    stop     - Send stop_clean command
    status   - Print current status once and exit
''')
        return
        
    rclpy.init()
    node = TestHelper(mode)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
