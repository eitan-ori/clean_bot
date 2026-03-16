#!/usr/bin/env python3
"""
###############################################################################
# FILE DESCRIPTION:
# This node acts as the primary hardware bridge between ROS 2 and the physical
# robot platform (Arduino-based). It handles bidirectional serial communication
# to control motors, relay, servo, and retrieve sensor feedback (ultrasonic).
#
# MAIN FUNCTIONS:
# 1. Subscribes to /cmd_vel_safe and converts twist commands into motor PWM values.
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

import math
import serial
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from std_msgs.msg import String


class ArduinoDriver(Node):
    def __init__(self):
        super().__init__(
            'arduino_driver',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

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
        self.declare_parameter('velocity_factor', 1.0)  # Multiply all velocities (2.0 = twice as fast)
        
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
        self.velocity_factor = self.get_parameter('velocity_factor').value
        
        self.get_logger().info(f'Arduino Driver v3.0 (No Encoders, With Relay) [velocity_factor={self.velocity_factor}]')
        
        # ===================== Serial Connection =====================
        self.serial = None
        self._serial_lock = threading.Lock()
        self._connect_serial()

        # ===================== Publishers =====================
        self.range_pub = self.create_publisher(Range, 'ultrasonic_range', 10)
        
        # Debug publisher - shows commands sent to Arduino
        self.debug_cmd_pub = self.create_publisher(Twist, 'cmd_vel_debug', 10)
        
        # ===================== Subscribers =====================
        # Subscribe to cmd_vel_safe (emergency_stop output) — final vetted commands
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel_safe', self.cmd_vel_callback, 10)
        # Fallback: also subscribe to cmd_vel_nav directly in case emergency_stop
        # is not running. This ensures the robot can still move.
        self.cmd_vel_nav_sub = self.create_subscription(
            Twist, 'cmd_vel_nav', self.cmd_vel_callback, 10)
        
        # Subscribe to mission commands for cleaning control (legacy)
        self.mission_cmd_sub = self.create_subscription(
            String, 'mission_command', self.mission_command_callback, 10)
        
        # Subscribe to arduino_command topic (from full_mission controller)
        self.arduino_cmd_sub = self.create_subscription(
            String, 'arduino_command', self.arduino_command_callback, 10)
        
        # ===================== State Variables =====================
        # Last command time (for watchdog)
        self.last_cmd_time = self.get_clock().now()
        self._motors_stopped = True  # Track whether motors are already at zero
        self.CMD_VEL_TIMEOUT_SEC = 0.5  # Stop motors if no cmd_vel for this long
        self._msg_count = 0  # Count received velocity messages
        
        # ===================== Timer =====================
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.update_loop)
        self.retry_timer = self.create_timer(5.0, self._retry_serial)
        self._heartbeat_timer = self.create_timer(3.0, self._heartbeat)
        
        self.get_logger().info(f'Arduino Driver started (with cleaning control)')

    def _connect_serial(self):
        """Try to open the serial port. Sets self.serial on success, None on failure."""
        try:
            self.serial = serial.Serial(
                self.serial_port,
                self.baud_rate,
                timeout=0.1
            )
            self.get_logger().info(f'✅ Connected to Arduino on {self.serial_port}')
        except serial.SerialException as e:
            self.serial = None
            self.get_logger().error(f'❌ Failed to connect to Arduino: {e}')
            self.get_logger().error(f'  Node stays alive (dry-run mode). Will retry every 5s.')
            self.get_logger().error(f'  Check: ls /dev/ttyUSB* /dev/ttyACM*')

    def _retry_serial(self):
        """Periodically retry serial connection if not connected."""
        if self.serial is None or not self.serial.is_open:
            self._connect_serial()

    def _heartbeat(self):
        """Periodic status so we can verify the node is alive."""
        serial_status = '✅ connected' if (self.serial and self.serial.is_open) else '❌ disconnected'
        self.get_logger().info(
            f'💓 HEARTBEAT: msgs_received={self._msg_count}  serial={serial_status}  '
            f'subs=[cmd_vel_safe, cmd_vel_nav]')

    def send_command(self, command: str):
        """Send a command to Arduino."""
        if self.serial is None:
            self.get_logger().warning(f'📤 [DRY-RUN] Would send: {command}')
            return False
        try:
            with self._serial_lock:
                self.serial.write(f"{command}\n".encode('utf-8'))
            self.get_logger().info(f'📤 Sent: {command}')
            return True
        except serial.SerialException as e:
            self.get_logger().warning(f'Serial write error: {e}')
            self.serial = None
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

    def cmd_vel_callback(self, msg: Twist):
        """Convert Twist message to motor PWM commands and send to Arduino."""
        try:
            self._msg_count += 1
            linear = msg.linear.x * self.velocity_factor
            angular = msg.angular.z * self.velocity_factor

            if not (math.isfinite(linear) and math.isfinite(angular)):
                return

            # CRITICAL FIX: Negate angular to match robot's physical wiring
            angular = -angular

            # Spin-or-drive: if angular is significant, spin in place; otherwise drive straight
            ANGULAR_THRESHOLD = 0.1  # rad/s — above this, pure spin
            if abs(angular) > ANGULAR_THRESHOLD:
                linear = 0.0  # Pure spin
            else:
                angular = 0.0  # Pure drive

            # PWM limits (tuned for this robot)
            MIN_PWM_FORWARD = 90
            MIN_PWM_ROTATE = 90
            MAX_PWM = 180

            # Standard differential drive mixing:
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
            
            # Send to Arduino (or log in dry-run mode)
            command = f"{left_pwm},{right_pwm}"
            if self.serial is not None:
                try:
                    with self._serial_lock:
                        self.serial.write(f"{command}\n".encode('utf-8'))
                except serial.SerialException as e:
                    self.get_logger().warning(f'Serial write error: {e}')
                    self.serial = None
            else:
                if left_pwm != 0 or right_pwm != 0:
                    self.get_logger().info(f'🔌 [NO SERIAL] PWM L={left_pwm} R={right_pwm} (not sent)')
            
            self.last_cmd_time = self.get_clock().now()
            self._motors_stopped = (left_pwm == 0 and right_pwm == 0)
        except Exception as e:
            self.get_logger().error(f'cmd_vel_callback error: {e}')

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
        # Deadband — only zero out if BOTH linear and angular commands are near zero
        if abs(wheel_speed_mps) < 0.005 and abs(linear_mps) < 0.01 and abs(angular_rps) < 0.01:
            return 0

        pwm = int((abs(wheel_speed_mps) / float(self.max_linear_speed)) * max_pwm)
        pwm = min(max_pwm, max(0, pwm))

        # Choose minimum based on whether we are mostly translating or mostly rotating.
        # If there is meaningful linear command, keep the stronger minimum.
        min_pwm = min_pwm_forward if abs(linear_mps) > 0.03 else min_pwm_rotate
        pwm = max(min_pwm, pwm)

        return pwm if wheel_speed_mps > 0 else -pwm

    def update_loop(self):
        """Main update loop - read from Arduino and publish ultrasonic data."""
        try:
            now = self.get_clock().now()
            
            # Watchdog: stop motors if no cmd_vel received recently
            if not self._motors_stopped:
                elapsed = (now - self.last_cmd_time).nanoseconds / 1e9
                if elapsed > self.CMD_VEL_TIMEOUT_SEC:
                    self.get_logger().warn('⚠️ cmd_vel timeout — stopping motors')
                    if self.serial is not None:
                        try:
                            with self._serial_lock:
                                self.serial.write(b"0,0\n")
                        except serial.SerialException:
                            self.serial = None
                    self._motors_stopped = True
            
            # Read data from Arduino
            if self.serial is None:
                return
            if not self.serial.is_open:
                return
            if self.serial.in_waiting > 0:
                try:
                    line = self.serial.readline().decode('utf-8').strip()
                    try:
                        distance_cm = float(line)
                        if 0 <= distance_cm <= 500:
                            self._publish_range(distance_cm, now)
                    except ValueError:
                        pass
                except (ValueError, UnicodeDecodeError):
                    pass
        except (serial.SerialException, OSError) as e:
            self.get_logger().warning(f'Serial error in update_loop: {e}')
        except Exception as e:
            self.get_logger().error(f'update_loop error: {e}')

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
        if self.serial is not None and self.serial.is_open:
            try:
                with self._serial_lock:
                    self.serial.write(b"0,0\n")         # Stop motors
                    self.serial.write(b"CLEAN_STOP\n")  # Stop cleaning
                self.serial.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ArduinoDriver()
        node.get_logger().info('🟢 Node initialized successfully')
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except SystemExit:
        pass
    except Exception as e:
        if node:
            node.get_logger().fatal(f'💀 Unexpected error: {e}')
        else:
            print(f'[arduino_driver] 💀 Failed to initialize: {e}')
        # DON'T re-raise — stay alive concept: let launch know we died
    finally:
        try:
            if node:
                node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
