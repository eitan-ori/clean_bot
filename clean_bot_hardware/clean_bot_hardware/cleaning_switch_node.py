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
from gpiozero import Servo, OutputDevice
from time import sleep

class CleaningSwitchNode(Node):
    def __init__(self):
        super().__init__('cleaning_switch_node')
        
        # ===================== Parameters =====================
        self.declare_parameter('servo_pin', 18)
        self.declare_parameter('relay_pin', 17)
        servo_pin = self.get_parameter('servo_pin').value
        relay_pin = self.get_parameter('relay_pin').value
        
        # ===================== Initialization =====================
        try:
            self.servo = Servo(servo_pin)
            self.relay = OutputDevice(relay_pin, active_high=True, initial_value=False)
            self.get_logger().info(f'✅ Servo on GPIO {servo_pin}, Relay on GPIO {relay_pin}')
        except Exception as e:
            self.get_logger().error(f'❌ Failed to initialize hardware: {e}')
            raise e

        # ===================== Subscribers =====================
        self.cmd_sub = self.create_subscription(
            Empty, 'mission_command', self.clean_callback, 10)

        
        self.get_logger().info('🧹 Cleaning Switch Node ready (with Relay Sequences)')

    def cmd_callback(self, msg):
        if msg.data == 'start_clean':
            self.get_logger().info('🔌 START SEQUENCE: Activating Gadget...')
            
            # 1. Servo Action
            self.servo.max()
            
            # 2. Relay Activation Pulse Sequence
            # Close for 0.5s -> Open for 0.5s -> Close for 0.5s -> Open
            self.relay.on()
            sleep(0.5)
            self.relay.off()
            sleep(0.5)
            self.relay.on()
            sleep(0.5)
            self.relay.off()
            
            sleep(0.5) # Wait for servo to finish
            self.servo.value = None # Stop jitter
        elif msg.data == 'stop_clean':
            self.get_logger().info('🔌 STOP SEQUENCE: Deactivating Gadget...')
            # 1. Servo Action
            self.servo.min()
            
            # 2. Relay Deactivation Pulse Sequence
            # Close for 1.0s -> Open
            self.relay.on()
            sleep(1.0)
            self.relay.off()
            
            sleep(0.5) # Wait for servo to finish
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
