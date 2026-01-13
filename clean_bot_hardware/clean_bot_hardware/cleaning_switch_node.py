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
        self.declare_parameter('relay_active_high', False) # Set to True if relay turns ON with 3.3V
        
        servo_pin = self.get_parameter('servo_pin').value
        relay_pin = self.get_parameter('relay_pin').value
        relay_active_high = self.get_parameter('relay_active_high').value
        
        self.get_logger().info(f'🔧 Init: servo={servo_pin}, relay={relay_pin}, active_high={relay_active_high}')
        self.get_logger().info(f'🔧 HAS_GPIO={HAS_GPIO}')
        
        # ===================== Hardware Initialization =====================
        self.servo = None
        self.relay = None
        
        if HAS_GPIO:
            try:
                self.get_logger().info(f'🔧 Creating Servo on GPIO {servo_pin}...')
                self.servo = Servo(servo_pin, initial_value=-1)
                self.get_logger().info(f'🔧 Servo created: {self.servo}')
                
                self.get_logger().info(f'🔧 Creating Relay on GPIO {relay_pin}...')
                # active_high determines if ON=1/OFF=0 (True) or ON=0/OFF=1 (False)
                # initial_value=False means start in "Logical OFF" state
                self.relay = OutputDevice(relay_pin, active_high=relay_active_high, initial_value=False)
                self.get_logger().info(f'🔧 Relay created: {self.relay}')
                
                self.get_logger().info('✅ Hardware initialized')
            except Exception as e:
                self.get_logger().error(f'⚠️ Failed to initialize GPIO hardware: {e}')
                import traceback
                self.get_logger().error(traceback.format_exc())
        else:
            self.get_logger().warn('⚠️ gpiozero not available - running in simulation mode')

        # ===================== Subscribers =====================
        self.cmd_sub = self.create_subscription(
            String, 'mission_command', self.cmd_callback, 10)
        
        self.get_logger().info('🧹 Cleaning Switch Node ready')
        self.get_logger().info('   Listening on /mission_command for start_clean/stop_clean')

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
        self.get_logger().info(f'🔧 activate_cleaning() called. servo={self.servo}, relay={self.relay}')
        
        if self.servo is None or self.relay is None:
            self.get_logger().warn('   [SIMULATION] Would activate servo and relay (hardware not available)')
            return
        
        try:
            # 1. Servo Action
            self.get_logger().info('🔧 Step 1: Moving servo to MIN position...')
            self.servo.min()
            self.get_logger().info(f'🔧 Servo value after min(): {self.servo.value}')
            
            # 2. Relay Activation Pulse Sequence (Matches Arduino "Start" Pattern)
            # Pattern: ON (2s) -> OFF (1s) -> ON (0.5s) -> OFF (1s)
            
            self.get_logger().info('🔧 Step 2: Relay ON for 2 seconds...')
            self.relay.on()
            sleep(2.0)
            
            self.get_logger().info('🔧 Step 3: Relay OFF for 1 second...')
            self.relay.off()
            sleep(1.0)
            
            self.get_logger().info('🔧 Step 4: Relay ON for 0.5 seconds...')
            self.relay.on()
            sleep(0.5)
            
            self.get_logger().info('🔧 Step 5: Relay OFF for 1 second (End of Start Sequence)...')
            self.relay.off()
            sleep(1.0)
            
            self.get_logger().info('🔧 Step 6: Stopping servo jitter...')
            self.servo.value = None  # Stop jitter
            
            self.get_logger().info('✅ Cleaning mechanism activated successfully!')
        except Exception as e:
            self.get_logger().error(f'❌ Failed to activate: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    def deactivate_cleaning(self):
        """Deactivate the cleaning mechanism."""
        self.get_logger().info(f'🔧 deactivate_cleaning() called. servo={self.servo}, relay={self.relay}')
        
        if self.servo is None or self.relay is None:
            self.get_logger().warn('   [SIMULATION] Would deactivate servo and relay (hardware not available)')
            return
        
        try:
            # 1. Servo Action
            self.get_logger().info('🔧 Step 1: Moving servo to Max position...')
            self.servo.max()
            self.get_logger().info(f'🔧 Servo value after max(): {self.servo.value}')
            
            # 2. Relay Deactivation Pulse Sequence
            self.get_logger().info('🔧 Step 2: Relay ON for 4 seconds...')
            self.relay.on()
            self.get_logger().info(f'🔧 Relay is_active: {self.relay.is_active}')
            sleep(4.0)
            
            self.get_logger().info('🔧 Step 3: Relay OFF (final)...')
            self.relay.off()
            
            self.get_logger().info('🔧 Step 4: Waiting 0.5s then stopping servo jitter...')
            sleep(0.5)
            self.servo.value = None  # Stop jitter
            
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
            pass  # Ignore shutdown errors (may already be shut down)


if __name__ == '__main__':
    main()
