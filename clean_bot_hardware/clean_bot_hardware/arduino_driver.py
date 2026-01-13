#!/usr/bin/env python3
"""
###############################################################################
# FILE DESCRIPTION:
# This node acts as the primary hardware bridge between ROS 2 and the physical
# robot platform (Arduino-based). It handles bidirectional serial communication
# to control motors, relay, servo, and retrieve sensor feedback (ultrasonic).
#
# MAIN FUNCTIONS:
# 1. Subscribes to /cmd_vel and converts twist commands into motor PWM values.
# 2. Receives and publishes ultrasonic range data for obstacle avoidance.
# 3. Handles cleaning commands (start_clean/stop_clean) - controls relay & servo.
#
# PARAMETERS & VALUES:
# - serial_port: /dev/ttyUSB0 (The USB port where Arduino is connected)
# - baud_rate: 57600 (Must match the Arduino sketch baud rate)
# - max_linear_speed: 0.3 m/s (Maximum speed target for PWM 255)
# - max_pwm: 255 (The maximum integer value for motor speed control)
# - publish_rate: 20.0 Hz (Frequency of the main control loop)
#
# ASSUMPTIONS:
# - The robot uses a differential drive configuration.
# - The Arduino is programmed with the matching communication protocol:
#   Input: "pwm_left,pwm_right\n" for motors
#   Input: "RELAY_ON\n" / "RELAY_OFF\n" for relay
#   Input: "SERVO_MIN\n" / "SERVO_MAX\n" for servo
#   Output: "distance_cm\n"
# - The user has serial port permissions (dialout group).
###############################################################################
"""

import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from std_msgs.msg import String
from time import sleep
import threading


class ArduinoDriver(Node):
    def __init__(self):
        super().__init__('arduino_driver')

        # ===================== Parameters =====================
        # Serial connection
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 57600)
        
        # Velocity conversion (for cmd_vel -> PWM)
        self.declare_parameter('max_linear_speed', 0.3)      # m/s at PWM 255
        self.declare_parameter('max_pwm', 255)
        
        # Frame IDs
        self.declare_parameter('ultrasonic_frame_id', 'ultrasonic_link')
        
        # Options
        self.declare_parameter('publish_rate', 20.0)  # Hz
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_pwm = self.get_parameter('max_pwm').value
        self.ultrasonic_frame = self.get_parameter('ultrasonic_frame_id').value
        publish_rate = self.get_parameter('publish_rate').value
        
        self.get_logger().info('Arduino Driver v3.0 (No Encoders, With Relay)')
        
        # ===================== Serial Connection =====================
        try:
            self.serial = serial.Serial(
                self.serial_port, 
                self.baud_rate, 
                timeout=0.1
            )
            self.get_logger().info(f'✅ Connected to Arduino on {self.serial_port}')
        except serial.SerialException as e:
            self.get_logger().error(f'❌ Failed to connect to Arduino: {e}')
            self.get_logger().error('Make sure:')
            self.get_logger().error('  1. Arduino is connected via USB')
            self.get_logger().error('  2. Serial port is correct (check with: ls /dev/ttyUSB* /dev/ttyACM*)')
            self.get_logger().error('  3. User has permission (sudo usermod -a -G dialout $USER)')
            raise SystemExit(1)

        # ===================== Publishers =====================
        self.range_pub = self.create_publisher(Range, 'ultrasonic_range', 10)
        
        # Debug publisher - shows commands sent to Arduino
        self.debug_cmd_pub = self.create_publisher(Twist, 'cmd_vel_debug', 10)
        
        # ===================== Subscribers =====================
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        # Subscribe to mission commands for cleaning control
        self.mission_cmd_sub = self.create_subscription(
            String, 'mission_command', self.mission_command_callback, 10)
        
        # ===================== State Variables =====================
        # Last command time (for watchdog)
        self.last_cmd_time = self.get_clock().now()
        
        # Cleaning sequence state
        self.cleaning_active = False
        
        # ===================== Timer =====================
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.update_loop)
        
        self.get_logger().info(f'Arduino Driver started (with cleaning control)')

    def send_command(self, command: str):
        """Send a command to Arduino."""
        try:
            self.serial.write(f"{command}\n".encode('utf-8'))
            self.get_logger().info(f'📤 Sent: {command}')
            return True
        except serial.SerialException as e:
            self.get_logger().warning(f'Serial write error: {e}')
            return False

    def mission_command_callback(self, msg: String):
        """Handle mission commands for cleaning control."""
        command = msg.data.lower().strip()
        self.get_logger().info(f'📬 Mission command: "{command}"')
        
        if command == 'start_clean':
            # Run cleaning sequence in separate thread to not block
            threading.Thread(target=self.activate_cleaning, daemon=True).start()
        elif command == 'stop_clean':
            threading.Thread(target=self.deactivate_cleaning, daemon=True).start()

    def activate_cleaning(self):
        """Activate the cleaning mechanism (servo + relay sequence)."""
        self.get_logger().info('🧹 START CLEANING SEQUENCE')
        self.cleaning_active = True
        
        try:
            # 1. Move servo to MIN position
            self.get_logger().info('   Step 1: Servo MIN')
            self.send_command('SERVO_MIN')
            
            # 2. Relay sequence: ON (2s) -> OFF (1s) -> ON (0.5s) -> OFF
            self.get_logger().info('   Step 2: Relay ON (2s)')
            self.send_command('RELAY_ON')
            sleep(2.0)
            
            self.get_logger().info('   Step 3: Relay OFF (1s)')
            self.send_command('RELAY_OFF')
            sleep(1.0)
            
            self.get_logger().info('   Step 4: Relay ON (0.5s)')
            self.send_command('RELAY_ON')
            sleep(0.5)
            
            self.get_logger().info('   Step 5: Relay OFF')
            self.send_command('RELAY_OFF')
            
            self.get_logger().info('✅ Cleaning activated!')
        except Exception as e:
            self.get_logger().error(f'❌ Cleaning activation failed: {e}')
        
        self.cleaning_active = False

    def deactivate_cleaning(self):
        """Deactivate the cleaning mechanism (servo + relay sequence)."""
        self.get_logger().info('🧹 STOP CLEANING SEQUENCE')
        self.cleaning_active = True
        
        try:
            # 1. Move servo to MAX position
            self.get_logger().info('   Step 1: Servo MAX')
            self.send_command('SERVO_MAX')
            
            # 2. Relay sequence: ON (4s) -> OFF
            self.get_logger().info('   Step 2: Relay ON (4s)')
            self.send_command('RELAY_ON')
            sleep(4.0)
            
            self.get_logger().info('   Step 3: Relay OFF')
            self.send_command('RELAY_OFF')
            
            self.get_logger().info('✅ Cleaning deactivated!')
        except Exception as e:
            self.get_logger().error(f'❌ Cleaning deactivation failed: {e}')
        
        self.cleaning_active = False

    def cmd_vel_callback(self, msg: Twist):
        """Convert Twist message to motor PWM commands and send to Arduino."""
        linear = msg.linear.x
        angular = msg.angular.z
        
        # Minimum PWM to overcome motor friction (adjust if motors still strain)
        MIN_PWM = 90
        MAX_PWM = 150  # Don't go too fast
        
        # Simplified movement: either rotate OR move forward
        # This prevents weird combined movements that confuse SLAM
        
        if abs(angular) > 0.1:
            # Pure rotation - spin in place
            # Positive angular = counter-clockwise = left backward, right forward
            rotation_pwm = int((abs(angular) / 1.0) * MAX_PWM)
            rotation_pwm = max(MIN_PWM, min(MAX_PWM, rotation_pwm))
            
            if angular > 0:
                # Counter-clockwise: left back, right forward
                left_pwm = rotation_pwm     # Left forward (will be negated = backward)
                right_pwm = -rotation_pwm   # Right backward (will be negated = forward)
            else:
                # Clockwise: left forward, right back
                left_pwm = -rotation_pwm
                right_pwm = rotation_pwm
                
        elif abs(linear) > 0.01:
            # Pure forward/backward motion - both wheels same speed
            forward_pwm = int((abs(linear) / self.max_linear_speed) * MAX_PWM)
            forward_pwm = max(MIN_PWM, min(MAX_PWM, forward_pwm))
            
            if linear > 0:
                # Forward: NEGATE because motors are inverted!
                left_pwm = -forward_pwm
                right_pwm = -forward_pwm
            else:
                # Backward
                left_pwm = forward_pwm
                right_pwm = forward_pwm
        else:
            # Stop
            left_pwm = 0
            right_pwm = 0
        
        # Publish debug info
        debug_msg = Twist()
        debug_msg.linear.x = float(left_pwm)
        debug_msg.linear.y = float(right_pwm)
        debug_msg.angular.z = angular
        self.debug_cmd_pub.publish(debug_msg)
        
        # Log significant commands
        if left_pwm != 0 or right_pwm != 0:
            self.get_logger().info(f'CMD: lin={linear:.2f} ang={angular:.2f} -> PWM L={left_pwm} R={right_pwm}')
        
        # Send to Arduino
        command = f"{left_pwm},{right_pwm}\n"
        try:
            self.serial.write(command.encode('utf-8'))
        except serial.SerialException as e:
            self.get_logger().warning(f'Serial write error: {e}')
        
        self.last_cmd_time = self.get_clock().now()

    def _velocity_to_pwm(self, velocity: float) -> int:
        """Convert velocity (m/s) to PWM value (-255 to 255)."""
        if abs(velocity) < 0.001:
            return 0
        
        # Linear mapping (could be improved with calibration curve)
        pwm = int((velocity / self.max_linear_speed) * self.max_pwm)
        
        # Minimum PWM threshold - motors won't move below ~90 PWM
        MIN_PWM = 90
        if 0 < pwm < MIN_PWM:
            pwm = MIN_PWM
        elif -MIN_PWM < pwm < 0:
            pwm = -MIN_PWM
        
        # Clamp to valid range
        return max(-self.max_pwm, min(self.max_pwm, pwm))

    def update_loop(self):
        """Main update loop - read from Arduino and publish ultrasonic data."""
        now = self.get_clock().now()
        
        # Read data from Arduino
        if self.serial.in_waiting > 0:
            try:
                line = self.serial.readline().decode('utf-8').strip()
                
                # Try to parse as distance (single number)
                try:
                    distance_cm = float(line)
                    self._publish_range(distance_cm, now)
                except ValueError:
                    pass  # Not a distance reading, ignore
                    
            except (ValueError, UnicodeDecodeError) as e:
                pass  # Ignore malformed data (common during startup)
            except serial.SerialException as e:
                self.get_logger().warning(f'Serial read error: {e}')

    def _publish_range(self, distance_cm: float, stamp):
        """Publish ultrasonic range measurement."""
        range_msg = Range()
        range_msg.header.stamp = stamp.to_msg()
        range_msg.header.frame_id = self.ultrasonic_frame
        range_msg.radiation_type = Range.ULTRASOUND
        range_msg.field_of_view = 0.26  # ~15 degrees
        range_msg.min_range = 0.02      # 2cm
        range_msg.max_range = 4.0       # 4m
        range_msg.range = distance_cm / 100.0  # Convert to meters
        
        self.range_pub.publish(range_msg)

    def destroy_node(self):
        """Clean shutdown - stop motors, relay, and servo."""
        if hasattr(self, 'serial') and self.serial.is_open:
            try:
                self.serial.write(b"0,0\n")        # Stop motors
                self.serial.write(b"RELAY_OFF\n")  # Ensure relay is off
                self.serial.write(b"SERVO_MIN\n")  # Return servo to start
                self.serial.close()
            except:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoDriver()
    
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
