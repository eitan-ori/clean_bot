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

        # Differential drive geometry
        self.declare_parameter('wheel_separation', 0.20)      # meters (distance between wheels)

        # Motor wiring/inversion (kept configurable)
        self.declare_parameter('invert_left_motor', True)
        self.declare_parameter('invert_right_motor', True)
        
        # Frame IDs
        self.declare_parameter('ultrasonic_frame_id', 'ultrasonic_link')
        
        # Options
        self.declare_parameter('publish_rate', 20.0)  # Hz
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_pwm = self.get_parameter('max_pwm').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.invert_left_motor = bool(self.get_parameter('invert_left_motor').value)
        self.invert_right_motor = bool(self.get_parameter('invert_right_motor').value)
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
        
        # Subscribe to mission commands for cleaning control (legacy)
        self.mission_cmd_sub = self.create_subscription(
            String, 'mission_command', self.mission_command_callback, 10)
        
        # Subscribe to arduino_command topic (from full_mission controller)
        self.arduino_cmd_sub = self.create_subscription(
            String, 'arduino_command', self.arduino_command_callback, 10)
        
        # ===================== State Variables =====================
        # Last command time (for watchdog)
        self.last_cmd_time = self.get_clock().now()

        # When True, Arduino runs its own autonomous wall-avoid logic.
        # In this mode we must not stream PWM commands from /cmd_vel.
        self.arduino_autonomous_active = False
        
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
        """Handle mission commands for cleaning control (legacy topic)."""
        command = msg.data.lower().strip()
        self.get_logger().info(f'📬 Mission command: "{command}"')
        
        if command == 'start_clean':
            self.get_logger().info('🧹 Sending CLEAN_START to Arduino')
            self.send_command('CLEAN_START')
        elif command == 'stop_clean':
            self.get_logger().info('🧹 Sending CLEAN_STOP to Arduino')
            self.send_command('CLEAN_STOP')
        elif command == 'auto_start':
            self.get_logger().info('🤖 Sending AUTO_START to Arduino')
            self.arduino_autonomous_active = True
            self.send_command('AUTO_START')
        elif command == 'auto_stop':
            self.get_logger().info('🤖 Sending AUTO_STOP to Arduino')
            self.arduino_autonomous_active = False
            self.send_command('AUTO_STOP')

    def arduino_command_callback(self, msg: String):
        """Handle arduino commands from full_mission controller."""
        command = msg.data.lower().strip()
        self.get_logger().info(f'📬 Arduino command: "{command}"')
        
        if command == 'start_clean':
            self.get_logger().info('🧹 Sending CLEAN_START to Arduino')
            self.send_command('CLEAN_START')
        elif command == 'stop_clean':
            self.get_logger().info('🧹 Sending CLEAN_STOP to Arduino')
            self.send_command('CLEAN_STOP')
        elif command == 'auto_start':
            self.get_logger().info('🤖 Sending AUTO_START to Arduino')
            self.arduino_autonomous_active = True
            self.send_command('AUTO_START')
        elif command == 'auto_stop':
            self.get_logger().info('🤖 Sending AUTO_STOP to Arduino')
            self.arduino_autonomous_active = False
            self.send_command('AUTO_STOP')

    def cmd_vel_callback(self, msg: Twist):
        """Convert Twist message to motor PWM commands and send to Arduino."""
        if self.arduino_autonomous_active:
            # Arduino is driving itself (wall-avoid). Do not override.
            return

        linear = msg.linear.x
        angular = msg.angular.z

        # PWM limits (tuned for this robot)
        MIN_PWM_FORWARD = 90  # Minimum to overcome friction
        MIN_PWM_ROTATE = 50   # Rotation can be slower
        MAX_PWM = 150         # Cap top speed

        # Differential drive mixing:
        # v_left  = v - w * (wheel_separation/2)
        # v_right = v + w * (wheel_separation/2)
        v_left = float(linear) - float(angular) * (self.wheel_separation / 2.0)
        v_right = float(linear) + float(angular) * (self.wheel_separation / 2.0)

        # Convert wheel speeds to PWM
        left_pwm = self._wheel_speed_to_pwm(v_left, linear, angular, MIN_PWM_FORWARD, MIN_PWM_ROTATE, MAX_PWM)
        right_pwm = self._wheel_speed_to_pwm(v_right, linear, angular, MIN_PWM_FORWARD, MIN_PWM_ROTATE, MAX_PWM)

        # Apply inversion per wheel (matches wiring)
        if self.invert_left_motor:
            left_pwm = -left_pwm
        if self.invert_right_motor:
            right_pwm = -right_pwm
        
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

    def _wheel_speed_to_pwm(
        self,
        wheel_speed_mps: float,
        linear_mps: float,
        angular_rps: float,
        min_pwm_forward: int,
        min_pwm_rotate: int,
        max_pwm: int,
    ) -> int:
        """Map wheel linear speed (m/s) to PWM.

        Uses a minimum PWM threshold so the motors actually move.
        """
        # Deadband
        if abs(wheel_speed_mps) < 0.01:
            return 0

        pwm = int((abs(wheel_speed_mps) / float(self.max_linear_speed)) * max_pwm)
        pwm = min(max_pwm, max(0, pwm))

        # Choose minimum based on whether we are mostly translating or mostly rotating.
        # If there is meaningful linear command, keep the stronger minimum.
        min_pwm = min_pwm_forward if abs(linear_mps) > 0.03 else min_pwm_rotate
        pwm = max(min_pwm, pwm)

        return pwm if wheel_speed_mps > 0 else -pwm

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
        """Clean shutdown - stop motors and cleaning system."""
        if hasattr(self, 'serial') and self.serial.is_open:
            try:
                self.serial.write(b"0,0\n")         # Stop motors
                self.serial.write(b"CLEAN_STOP\n")  # Stop cleaning
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
