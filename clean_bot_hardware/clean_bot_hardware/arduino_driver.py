#!/usr/bin/env python3
"""
###############################################################################
# FILE DESCRIPTION:
# This node acts as the primary hardware bridge between ROS 2 and the physical
# robot platform (Arduino-based). It handles bidirectional serial communication
# to control motors and retrieve sensor feedback (encoders and ultrasonic).
#
# MAIN FUNCTIONS:
# 1. Subscribes to /cmd_vel and converts twist commands into motor PWM values.
# 2. Receives encoder ticks from Arduino and calculates wheel odometry.
# 3. Publishes odometry data and TF transforms (odom -> base_link).
# 4. Receives and publishes ultrasonic range data for obstacle avoidance.
#
# PARAMETERS & VALUES:
# - serial_port: /dev/ttyUSB0 (The USB port where Arduino is connected)
# - baud_rate: 57600 (Must match the Arduino sketch baud rate)
# - wheel_radius: 0.0335 m (Physical radius of the drive wheels)
# - wheel_separation: 0.20 m (Distance between the center of left and right wheels)
# - ticks_per_revolution: 1320 (Total encoder counts for one full wheel rotation)
# - max_linear_speed: 0.3 m/s (Maximum speed target for PWM 255)
# - max_pwm: 255 (The maximum integer value for motor speed control)
# - publish_rate: 20.0 Hz (Frequency of the main control loop)
#
# ASSUMPTIONS:
# - The robot uses a differential drive configuration.
# - The Arduino is programmed with the matching communication protocol:
#   Input: "pwm_left,pwm_right\n"
#   Output: "left_ticks,right_ticks,distance_cm\n"
# - The user has serial port permissions (dialout group).
###############################################################################
"""

import math
import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from tf2_ros import TransformBroadcaster


class ArduinoDriver(Node):
    def __init__(self):
        super().__init__('arduino_driver')

        # ===================== Parameters =====================
        # Serial connection
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 57600)
        
        # Robot physical parameters (MUST BE CALIBRATED!)
        self.declare_parameter('wheel_radius', 0.0335)        # GB37-131 wheel radius (meters)
        self.declare_parameter('wheel_separation', 0.20)     # Distance between wheels (meters)
        self.declare_parameter('ticks_per_revolution', 1320) # Encoder CPR * Gear ratio (11 * 120)
        
        # Velocity conversion (for cmd_vel -> PWM)
        self.declare_parameter('max_linear_speed', 0.3)      # m/s at PWM 255
        self.declare_parameter('max_pwm', 255)
        
        # Frame IDs
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('ultrasonic_frame_id', 'ultrasonic_link')
        
        # Options
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_rate', 20.0)  # Hz
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.ticks_per_rev = self.get_parameter('ticks_per_revolution').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_pwm = self.get_parameter('max_pwm').value
        self.odom_frame = self.get_parameter('odom_frame_id').value
        self.base_frame = self.get_parameter('base_frame_id').value
        self.ultrasonic_frame = self.get_parameter('ultrasonic_frame_id').value
        self.publish_tf = self.get_parameter('publish_tf').value
        publish_rate = self.get_parameter('publish_rate').value
        
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
        self.odom_pub = self.create_publisher(Odometry, 'wheel_odom', 10)
        self.range_pub = self.create_publisher(Range, 'ultrasonic_range', 10)
        
        # ===================== Subscribers =====================
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        # ===================== TF Broadcaster =====================
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
        
        # ===================== State Variables =====================
        # Position tracking
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Encoder tracking
        self.prev_left_ticks = None
        self.prev_right_ticks = None
        
        # Velocity tracking (for odometry message)
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        
        # Last command time (for watchdog)
        self.last_cmd_time = self.get_clock().now()
        
        # ===================== Timer =====================
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.update_loop)
        
        self.get_logger().info(f'Arduino Driver started:')
        self.get_logger().info(f'  Wheel radius: {self.wheel_radius}m')
        self.get_logger().info(f'  Wheel separation: {self.wheel_separation}m')
        self.get_logger().info(f'  Ticks/rev: {self.ticks_per_rev}')

    def cmd_vel_callback(self, msg: Twist):
        """Convert Twist message to motor PWM commands and send to Arduino."""
        linear = msg.linear.x
        angular = msg.angular.z
        
        # Differential drive kinematics
        left_vel = linear - (angular * self.wheel_separation / 2.0)
        right_vel = linear + (angular * self.wheel_separation / 2.0)
        
        # Convert m/s to PWM (-255 to 255)
        left_pwm = self._velocity_to_pwm(left_vel)
        right_pwm = self._velocity_to_pwm(right_vel)
        
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
        
        # Clamp to valid range
        return max(-self.max_pwm, min(self.max_pwm, pwm))

    def update_loop(self):
        """Main update loop - read from Arduino and publish odometry."""
        now = self.get_clock().now()
        
        # Read data from Arduino
        if self.serial.in_waiting > 0:
            try:
                line = self.serial.readline().decode('utf-8').strip()
                parts = line.split(',')
                
                if len(parts) == 3:
                    left_ticks = int(parts[0])
                    right_ticks = int(parts[1])
                    distance_cm = float(parts[2])
                    
                    # Update odometry
                    self._update_odometry(left_ticks, right_ticks, now)
                    
                    # Publish ultrasonic range
                    self._publish_range(distance_cm, now)
                    
            except (ValueError, UnicodeDecodeError) as e:
                pass  # Ignore malformed data (common during startup)
            except serial.SerialException as e:
                self.get_logger().warning(f'Serial read error: {e}')

    def _update_odometry(self, left_ticks: int, right_ticks: int, stamp):
        """Calculate and publish wheel odometry."""
        # Initialize previous ticks on first call
        if self.prev_left_ticks is None:
            self.prev_left_ticks = left_ticks
            self.prev_right_ticks = right_ticks
            return
        
        # Calculate tick deltas
        delta_left = left_ticks - self.prev_left_ticks
        delta_right = right_ticks - self.prev_right_ticks
        
        self.prev_left_ticks = left_ticks
        self.prev_right_ticks = right_ticks
        
        # Convert ticks to distance traveled (meters)
        meters_per_tick = (2.0 * math.pi * self.wheel_radius) / self.ticks_per_rev
        left_dist = delta_left * meters_per_tick
        right_dist = delta_right * meters_per_tick
        
        # Calculate robot motion
        center_dist = (left_dist + right_dist) / 2.0
        delta_theta = (right_dist - left_dist) / self.wheel_separation
        
        # Update position (integrate)
        self.x += center_dist * math.cos(self.theta + delta_theta / 2.0)
        self.y += center_dist * math.sin(self.theta + delta_theta / 2.0)
        self.theta += delta_theta
        
        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Estimate velocities (rough approximation)
        dt = 0.05  # Assume 50ms between updates (from Arduino)
        self.linear_vel = center_dist / dt
        self.angular_vel = delta_theta / dt
        
        # Create and publish Odometry message
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        
        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = self._yaw_to_quaternion(self.theta)
        
        # Velocity
        odom.twist.twist.linear.x = self.linear_vel
        odom.twist.twist.angular.z = self.angular_vel
        
        # Covariance (simple diagonal)
        odom.pose.covariance[0] = 0.01   # x
        odom.pose.covariance[7] = 0.01   # y
        odom.pose.covariance[35] = 0.05  # yaw
        odom.twist.covariance[0] = 0.01
        odom.twist.covariance[35] = 0.05
        
        self.odom_pub.publish(odom)
        
        # Publish TF
        if self.publish_tf:
            self._publish_tf(stamp)

    def _publish_tf(self, stamp):
        """Publish odom -> base_link transform."""
        t = TransformStamped()
        t.header.stamp = stamp.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = self._yaw_to_quaternion(self.theta)
        
        self.tf_broadcaster.sendTransform(t)

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

    def _yaw_to_quaternion(self, yaw: float) -> Quaternion:
        """Convert yaw angle to quaternion."""
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def destroy_node(self):
        """Clean shutdown - stop motors."""
        if hasattr(self, 'serial') and self.serial.is_open:
            try:
                self.serial.write(b"0,0\n")  # Stop motors
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
