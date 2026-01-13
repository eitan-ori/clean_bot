#!/usr/bin/env python3
"""
Simple Boustrophedon (Plowing) Coverage - Ultra Simple Version
==============================================================
מתנועע בצורת חרישה פשוטה - קדימה, סיבוב, קדימה, סיבוב...

הנחות:
- החדר בערך ריבועי או L-shape
- הרובוט מתחיל מנקודת התחלה (0,0)
- פשוט הולך קדימה עד שפוגע במכשול, מסתובב, וממשיך
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from sensor_msgs.msg import LaserScan


class SimpleCoverage(Node):
    """
    תנועת חרישה פשוטה:
    1. נסע קדימה
    2. כשמגיע לקיר - עצור
    3. זוז הצידה (צעד אחד)
    4. סובב 180°
    5. חזור ל-1
    """
    
    def __init__(self):
        super().__init__('simple_coverage')
        
        # === פרמטרים ===
        self.declare_parameter('linear_speed', 0.12)      # מהירות נסיעה (m/s)
        self.declare_parameter('angular_speed', 0.3)      # מהירות סיבוב (rad/s)
        self.declare_parameter('row_spacing', 0.12)       # רוחב בין שורות (מטר)
        self.declare_parameter('wall_distance', 0.25)     # מרחק עצירה מקיר (מטר)
        self.declare_parameter('side_step_distance', 0.12) # צעד הצידה (מטר)
        
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.row_spacing = self.get_parameter('row_spacing').value
        self.wall_distance = self.get_parameter('wall_distance').value
        self.side_step_distance = self.get_parameter('side_step_distance').value
        
        # === Publishers ===
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.state_pub = self.create_publisher(String, 'coverage_state', 10)
        self.complete_pub = self.create_publisher(Bool, 'coverage_complete', 10)
        
        # === Subscribers ===
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10)
        self.control_sub = self.create_subscription(
            String, 'coverage_control', self.control_callback, 10)
        
        # === State ===
        self.state = 'IDLE'  # IDLE, DRIVING, TURNING, SIDE_STEP, COMPLETE
        self.running = False
        
        # Robot pose
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        
        # Scan data
        self.front_distance = 10.0  # Distance to obstacle in front
        self.min_front_distance = 10.0
        
        # Movement targets
        self.target_yaw = 0.0
        self.target_x = 0.0
        self.target_y = 0.0
        self.side_step_start_x = 0.0
        self.side_step_start_y = 0.0
        
        # Direction: 1 = forward (+X), -1 = backward (-X)
        self.direction = 1
        self.row_count = 0
        self.max_rows = 50  # Safety limit
        
        # Timer for main control loop
        self.timer = self.create_timer(0.05, self.control_loop)  # 20Hz
        
        self.get_logger().info('=' * 50)
        self.get_logger().info('🧹 Simple Coverage Started')
        self.get_logger().info(f'   Speed: {self.linear_speed} m/s')
        self.get_logger().info(f'   Row spacing: {self.row_spacing} m')
        self.get_logger().info(f'   Wall distance: {self.wall_distance} m')
        self.get_logger().info('   Send "start" to /coverage_control to begin')
        self.get_logger().info('=' * 50)

    # ==================== Callbacks ====================
    
    def odom_callback(self, msg: Odometry):
        """Update robot position from odometry."""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)

    def scan_callback(self, msg: LaserScan):
        """Update front distance from laser scan."""
        # Get distances in front of robot (-30° to +30°)
        num_readings = len(msg.ranges)
        if num_readings == 0:
            return
            
        # Front is at index num_readings/2 for 360° scan
        # Or index 0 for forward-facing scan
        center = num_readings // 2
        
        # Check front 60° (30° each side)
        angle_per_reading = (msg.angle_max - msg.angle_min) / num_readings
        front_readings = int(math.radians(60) / angle_per_reading / 2)
        
        start_idx = max(0, center - front_readings)
        end_idx = min(num_readings, center + front_readings)
        
        # Get minimum distance in front
        front_ranges = []
        for i in range(start_idx, end_idx):
            r = msg.ranges[i]
            if msg.range_min < r < msg.range_max:
                front_ranges.append(r)
        
        if front_ranges:
            self.front_distance = min(front_ranges)
            self.min_front_distance = min(self.min_front_distance, self.front_distance)
        else:
            self.front_distance = 10.0

    def control_callback(self, msg: String):
        """Handle control commands."""
        cmd = msg.data.lower().strip()
        self.get_logger().info(f'📬 Received command: {cmd}')
        
        if cmd == 'start':
            self.start_coverage()
        elif cmd == 'stop':
            self.stop_coverage()
        elif cmd == 'pause':
            self.pause_coverage()
        elif cmd == 'resume':
            self.resume_coverage()

    # ==================== Control ====================
    
    def start_coverage(self):
        """Start the coverage mission."""
        if self.state == 'IDLE' or self.state == 'COMPLETE':
            self.get_logger().info('▶️ Starting coverage!')
            self.running = True
            self.state = 'DRIVING'
            self.direction = 1
            self.row_count = 0
            self.target_yaw = 0.0  # Start facing +X
            self.publish_state()

    def stop_coverage(self):
        """Stop the coverage mission."""
        self.get_logger().info('🛑 Stopping coverage')
        self.running = False
        self.state = 'IDLE'
        self.stop_robot()
        self.publish_state()

    def pause_coverage(self):
        """Pause the coverage mission."""
        self.get_logger().info('⏸️ Pausing coverage')
        self.running = False
        self.stop_robot()

    def resume_coverage(self):
        """Resume the coverage mission."""
        if self.state != 'IDLE' and self.state != 'COMPLETE':
            self.get_logger().info('▶️ Resuming coverage')
            self.running = True

    # ==================== Main Control Loop ====================
    
    def control_loop(self):
        """Main control loop - runs at 20Hz."""
        if not self.running:
            return
        
        if self.state == 'DRIVING':
            self.do_driving()
        elif self.state == 'TURNING':
            self.do_turning()
        elif self.state == 'SIDE_STEP':
            self.do_side_step()
        elif self.state == 'COMPLETE':
            self.running = False
            self.stop_robot()

    def do_driving(self):
        """Drive forward until hitting a wall."""
        # Check if we hit a wall
        if self.front_distance < self.wall_distance:
            self.get_logger().info(f'🧱 Wall detected at {self.front_distance:.2f}m - starting side step')
            self.stop_robot()
            
            # Check if we've done enough rows
            if self.row_count >= self.max_rows:
                self.complete_coverage()
                return
            
            # Start side step
            self.state = 'SIDE_STEP'
            self.side_step_start_x = self.x
            self.side_step_start_y = self.y
            return
        
        # Drive forward
        cmd = Twist()
        cmd.linear.x = self.linear_speed
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

    def do_side_step(self):
        """Move sideways (actually: turn 90°, move, turn back)."""
        # Calculate how far we've moved from start of side step
        dx = self.x - self.side_step_start_x
        dy = self.y - self.side_step_start_y
        distance_moved = math.sqrt(dx*dx + dy*dy)
        
        if distance_moved >= self.side_step_distance:
            # Side step complete - now turn 180° for next row
            self.get_logger().info(f'↔️ Side step complete - turning 180°')
            self.stop_robot()
            
            # Set target yaw (opposite direction)
            if self.direction == 1:
                self.target_yaw = math.pi  # Face -X
                self.direction = -1
            else:
                self.target_yaw = 0.0  # Face +X
                self.direction = 1
            
            self.row_count += 1
            self.state = 'TURNING'
            return
        
        # Move in +Y direction (perpendicular to main travel)
        cmd = Twist()
        # First turn to face +Y
        target_side = math.pi / 2  # +Y direction
        angle_error = self.normalize_angle(target_side - self.yaw)
        
        if abs(angle_error) > 0.1:
            # Still turning to face side
            cmd.linear.x = 0.0
            cmd.angular.z = 0.3 if angle_error > 0 else -0.3
        else:
            # Facing side - drive
            cmd.linear.x = self.linear_speed
            cmd.angular.z = 0.0
        
        self.cmd_vel_pub.publish(cmd)

    def do_turning(self):
        """Turn to face the target direction."""
        angle_error = self.normalize_angle(self.target_yaw - self.yaw)
        
        if abs(angle_error) < 0.15:  # ~8 degrees tolerance
            # Turn complete - start driving
            self.get_logger().info(f'🔄 Turn complete - row {self.row_count}, driving...')
            self.stop_robot()
            self.min_front_distance = 10.0  # Reset front distance tracking
            self.state = 'DRIVING'
            return
        
        # Turn towards target
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = self.angular_speed if angle_error > 0 else -self.angular_speed
        self.cmd_vel_pub.publish(cmd)

    def complete_coverage(self):
        """Coverage complete."""
        self.get_logger().info('=' * 50)
        self.get_logger().info('✅ COVERAGE COMPLETE!')
        self.get_logger().info(f'   Rows completed: {self.row_count}')
        self.get_logger().info('=' * 50)
        
        self.state = 'COMPLETE'
        self.running = False
        self.stop_robot()
        
        # Publish completion
        msg = Bool()
        msg.data = True
        self.complete_pub.publish(msg)
        self.publish_state()

    # ==================== Helpers ====================
    
    def stop_robot(self):
        """Stop the robot."""
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def publish_state(self):
        """Publish current state."""
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleCoverage()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
