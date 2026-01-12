#!/usr/bin/env python3
"""
###############################################################################
# FILE DESCRIPTION:
# This node manages the physical cleaning switch via a servo motor.
# It listens to mission_command topic for start_clean/stop_clean commands.
#
# MAIN FUNCTIONS:
# 1. Subscribes to /mission_command (std_msgs/String)
# 2. On "start_clean": Activates servo and relay sequence
# 3. On "stop_clean": Deactivates servo and relay sequence
#
# ASSUMPTIONS:
# - Servo signal is on GPIO 18.
# - Relay is on GPIO 8.
# - gpiozero is installed on the Raspberry Pi.
###############################################################################
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from time import sleep

# Try to import gpiozero - may not be available on non-Pi systems
try:
    from gpiozero import Servo, OutputDevice
    HAS_GPIO = True
except ImportError:
    HAS_GPIO = False


class CleaningSwitchNode(Node):
    def __init__(self):
        super().__init__('cleaning_switch_node')
        
        # ===================== Parameters =====================
        self.declare_parameter('servo_pin', 18)
        self.declare_parameter('relay_pin', 17)
        servo_pin = self.get_parameter('servo_pin').value
        relay_pin = self.get_parameter('relay_pin').value
        
        # ===================== Hardware Initialization =====================
        self.servo = None
        self.relay = None
        
        if HAS_GPIO:
            try:
                self.servo = Servo(servo_pin)
                self.relay = OutputDevice(relay_pin, active_high=True, initial_value=False)
                self.get_logger().info(f'✅ Servo on GPIO {servo_pin}, Relay on GPIO {relay_pin}')
            except Exception as e:
                self.get_logger().warn(f'⚠️ Failed to initialize GPIO hardware: {e}')
                self.get_logger().warn('   Running in simulation mode (no hardware control)')
        else:
            self.get_logger().warn('⚠️ gpiozero not available - running in simulation mode')

        # ===================== Subscribers =====================
        self.cmd_sub = self.create_subscription(
            String, 'mission_command', self.cmd_callback, 10)
        
        self.get_logger().info('🧹 Cleaning Switch Node ready')
        self.get_logger().info('   Listening on /mission_command for start_clean/stop_clean')

    def cmd_callback(self, msg):
        command = msg.data.lower().strip()
        
        if command == 'start_clean':
            self.get_logger().info('🔌 START SEQUENCE: Activating Gadget...')
            self.activate_cleaning()
        elif command == 'stop_clean':
            self.get_logger().info('🔌 STOP SEQUENCE: Deactivating Gadget...')
            self.deactivate_cleaning()
        # Ignore other commands (they're handled by full_mission_controller)

    def activate_cleaning(self):
        """Activate the cleaning mechanism."""
        if self.servo is None or self.relay is None:
            self.get_logger().info('   [SIMULATION] Would activate servo and relay')
            return
        
        try:
            # 1. Servo Action
            self.servo.max()
            
            # 2. Relay Activation Pulse Sequence
            self.relay.on()
            sleep(2)
            self.relay.off()
            sleep(0.5)
            self.relay.on()
            sleep(0.5)
            self.relay.off()
            
            sleep(0.5)  # Wait for servo to finish
            self.servo.value = None  # Stop jitter
            
            self.get_logger().info('✅ Cleaning mechanism activated')
        except Exception as e:
            self.get_logger().error(f'❌ Failed to activate: {e}')

    def deactivate_cleaning(self):
        """Deactivate the cleaning mechanism."""
        if self.servo is None or self.relay is None:
            self.get_logger().info('   [SIMULATION] Would deactivate servo and relay')
            return
        
        try:
            # 1. Servo Action
            self.servo.min()
            
            # 2. Relay Deactivation Pulse Sequence
            self.relay.on()
            sleep(3.0)
            self.relay.off()
            
            sleep(0.5)  # Wait for servo to finish
            self.servo.value = None  # Stop jitter
            
            self.get_logger().info('✅ Cleaning mechanism deactivated')
        except Exception as e:
            self.get_logger().error(f'❌ Failed to deactivate: {e}')


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
