#!/usr/bin/env python3
"""
###############################################################################
# FILE DESCRIPTION:
# This node manages the physical cleaning switch via a servo motor.
# It listens to specific ROS 2 topics to turn the switch on and off.
#
# MAIN FUNCTIONS:
# 1. Subscribes to /clean (std_msgs/Empty): Switches ON.
# 2. Subscribes to /stop_clean (std_msgs/Empty): Switches OFF.
# 3. Controls a servo on GPIO 18 using gpiozero.
#
# ASSUMPTIONS:
# - Servo signal (yellow) is on GPIO 18.
# - gpiozero is installed on the Raspberry Pi.
###############################################################################
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from gpiozero import Servo
from time import sleep

class CleaningSwitchNode(Node):
    def __init__(self):
        super().__init__('cleaning_switch_node')
        
        # ===================== Parameters =====================
        self.declare_parameter('servo_pin', 18)
        servo_pin = self.get_parameter('servo_pin').value
        
        # ===================== Initialization =====================
        try:
            self.servo = Servo(servo_pin)
            self.get_logger().info(f'✅ Servo initialized on GPIO {servo_pin}')
        except Exception as e:
            self.get_logger().error(f'❌ Failed to initialize servo: {e}')
            raise e

        # ===================== Subscribers =====================
        self.clean_sub = self.create_subscription(
            Empty, 'clean', self.clean_callback, 10)
        self.stop_sub = self.create_subscription(
            Empty, 'stop_clean_relay', self.stop_callback, 10)
        
        self.get_logger().info('🧹 Cleaning Switch Node ready')

    def clean_callback(self, msg):
        self.get_logger().info('🔌 ACTIVATE: Turning switch ON...')
        self.servo.max()
        sleep(1.0)
        self.servo.value = None # Stop jitter

    def stop_callback(self, msg):
        self.get_logger().info('🔌 DEACTIVATE: Turning switch OFF...')
        self.servo.min()
        sleep(1.0)
        self.servo.value = None # Stop jitter

def main(args=None):
    rclpy.init(args=args)
    node = CleaningSwitchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
