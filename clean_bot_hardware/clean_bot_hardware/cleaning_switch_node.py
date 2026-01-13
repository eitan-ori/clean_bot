#!/usr/bin/env python3
"""
###############################################################################
# FILE DESCRIPTION:
# This node manages the physical cleaning switch via a servo motor (GPIO)
# and relay (controlled via Arduino through the arduino_driver bridge).
#
# MAIN FUNCTIONS:
# 1. Subscribes to /mission_command (std_msgs/String)
# 2. On "start_clean": Activates servo (GPIO) and publishes relay commands
# 3. On "stop_clean": Deactivates servo (GPIO) and publishes relay commands
#
# COMMUNICATION:
# - Servo: Direct GPIO control (pin 18)
# - Relay: Publishes to /relay_command topic (arduino_driver handles serial)
###############################################################################
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from time import sleep

# Try to import gpiozero - may not be available on non-Pi systems
try:
    from gpiozero import Servo
    HAS_GPIO = True
except ImportError:
    HAS_GPIO = False


class CleaningSwitchNode(Node):
    def __init__(self):
        super().__init__('cleaning_switch_node')
        
        # ===================== Parameters =====================
        self.declare_parameter('servo_pin', 18)
        servo_pin = self.get_parameter('servo_pin').value
        
        self.get_logger().info(f'🔧 Init: servo_pin={servo_pin}')
        self.get_logger().info(f'🔧 HAS_GPIO={HAS_GPIO}')
        
        # ===================== Hardware Initialization =====================
        self.servo = None
        
        # Initialize Servo (GPIO)
        if HAS_GPIO:
            try:
                self.get_logger().info(f'🔧 Creating Servo on GPIO {servo_pin}...')
                self.servo = Servo(servo_pin, initial_value=-1)
                self.get_logger().info(f'✅ Servo created')
            except Exception as e:
                self.get_logger().error(f'⚠️ Failed to initialize Servo: {e}')
        else:
            self.get_logger().warn('⚠️ gpiozero not available - servo in simulation mode')

        # ===================== Publishers =====================
        # Relay commands go to arduino_driver via this topic
        self.relay_pub = self.create_publisher(String, 'relay_command', 10)
        
        # ===================== Subscribers =====================
        self.cmd_sub = self.create_subscription(
            String, 'mission_command', self.cmd_callback, 10)
        
        self.get_logger().info('🧹 Cleaning Switch Node ready')
        self.get_logger().info('   Listening on /mission_command for start_clean/stop_clean')
        self.get_logger().info('   Publishing relay commands to /relay_command')

    def send_relay_command(self, command: str):
        """Publish a relay command to the arduino_driver bridge."""
        msg = String()
        msg.data = command
        self.relay_pub.publish(msg)
        self.get_logger().info(f'📤 Published relay command: {command}')

    def cmd_callback(self, msg):
        command = msg.data.lower().strip()
        self.get_logger().info(f'📬 Received command: "{command}"')
        
        if command == 'start_clean':
            self.get_logger().info('🔌 START SEQUENCE: Activating Gadget...')
            self.activate_cleaning()
        elif command == 'stop_clean':
            self.get_logger().info('🔌 STOP SEQUENCE: Deactivating Gadget...')
            self.deactivate_cleaning()
        else:
            self.get_logger().info(f'   (Ignoring command "{command}" - not for me)')

    def activate_cleaning(self):
        """Activate the cleaning mechanism."""
        self.get_logger().info('🔧 activate_cleaning() called')
        
        try:
            # 1. Servo Action (GPIO)
            if self.servo:
                self.get_logger().info('🔧 Step 1: Moving servo to MIN position...')
                self.servo.min()
            else:
                self.get_logger().warn('   [SIMULATION] Would move servo to MIN')
            
            # 2. Relay Activation Pulse Sequence via Arduino
            # Pattern: ON (2s) -> OFF (1s) -> ON (0.5s) -> OFF (1s)
            
            self.get_logger().info('🔧 Step 2: Relay ON for 2 seconds...')
            self.send_relay_command('RELAY_ON')
            sleep(2.0)
            
            self.get_logger().info('🔧 Step 3: Relay OFF for 1 second...')
            self.send_relay_command('RELAY_OFF')
            sleep(1.0)
            
            self.get_logger().info('🔧 Step 4: Relay ON for 0.5 seconds...')
            self.send_relay_command('RELAY_ON')
            sleep(0.5)
            
            self.get_logger().info('🔧 Step 5: Relay OFF (End of Start Sequence)...')
            self.send_relay_command('RELAY_OFF')
            sleep(1.0)
            
            # 3. Stop servo jitter
            if self.servo:
                self.get_logger().info('🔧 Step 6: Stopping servo jitter...')
                self.servo.value = None
            
            self.get_logger().info('✅ Cleaning mechanism activated successfully!')
        except Exception as e:
            self.get_logger().error(f'❌ Failed to activate: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    def deactivate_cleaning(self):
        """Deactivate the cleaning mechanism."""
        self.get_logger().info('🔧 deactivate_cleaning() called')
        
        try:
            # 1. Servo Action (GPIO)
            if self.servo:
                self.get_logger().info('🔧 Step 1: Moving servo to MAX position...')
                self.servo.max()
            else:
                self.get_logger().warn('   [SIMULATION] Would move servo to MAX')
            
            # 2. Relay Deactivation Pulse Sequence via Arduino
            # Pattern: ON (4s) -> OFF
            
            self.get_logger().info('🔧 Step 2: Relay ON for 4 seconds...')
            self.send_relay_command('RELAY_ON')
            sleep(4.0)
            
            self.get_logger().info('🔧 Step 3: Relay OFF (final)...')
            self.send_relay_command('RELAY_OFF')
            
            # 3. Stop servo jitter
            if self.servo:
                self.get_logger().info('🔧 Step 4: Stopping servo jitter...')
                sleep(0.5)
                self.servo.value = None
            
            self.get_logger().info('✅ Cleaning mechanism deactivated successfully!')
        except Exception as e:
            self.get_logger().error(f'❌ Failed to deactivate: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())


def main(args=None):
    rclpy.init(args=args)
    node = CleaningSwitchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
