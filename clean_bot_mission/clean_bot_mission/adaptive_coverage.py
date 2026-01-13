#!/usr/bin/env python3
"""
###############################################################################
# FILE DESCRIPTION:
# This node implements an advanced adaptive coverage planner for irregular
# and complex indoor environments. Unlike simple rectangular planners, it
# uses cell decomposition to break the free space into segments and applies
# optimized zigzag paths to each segment.
#
# MAIN FUNCTIONS:
# 1. Analyzes the current occupancy grid map and inflates obstacles to 
#    account for robot footprint.
# 2. Decomposes the floor into convex or near-convex cells.
# 3. Calculates a coverage path (zigzag) within each cell, respecting the 
#    robot's cleaning tool width.
# 4. Sequences goals to Nav2 to execute the full coverage mission.
#
# CONTROL (via /coverage_control topic, std_msgs/String):
# - "start"  : Start/resume coverage
# - "stop"   : Stop coverage
# - "pause"  : Pause coverage (can resume)
# - "resume" : Resume from pause
# - "reset"  : Reset coverage state
#
# PARAMETERS & VALUES:
# - coverage_width: 0.14 m (Effective cleaning width of the robot).
# - overlap_ratio: 0.15 (15% overlap between cleaning passes).
# - robot_radius: 0.12 m (Used to avoid clipping walls/furniture).
# - min_region_area: 0.1 m² (Ignore very small pockets of space).
# - start_on_exploration_complete: True (Auto-trigger after mapping).
#
# ASSUMPTIONS:
# - Scipy and Numpy are available for image processing/array operations.
# - The map is accurate and represents the current physical state.
# - The robot can execute point-to-point navigation reliably.
###############################################################################
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray

from scipy import ndimage
from collections import deque
import heapq


# Occupancy grid values
FREE_THRESHOLD = 50
OCCUPIED_THRESHOLD = 65


class CoverageState:
    """Coverage state enumeration."""
    IDLE = 'IDLE'           # Waiting for start command
    RUNNING = 'RUNNING'     # Actively covering
    PAUSED = 'PAUSED'       # Temporarily paused
    STOPPED = 'STOPPED'     # Stopped by user command
    COMPLETE = 'COMPLETE'   # Coverage finished


class AdaptiveCoveragePlanner(Node):
    def __init__(self):
        super().__init__('adaptive_coverage_planner')

        # ===================== Parameters =====================
        self.declare_parameter('coverage_width', 0.14)           # 14cm cleaning width (main suction)
        self.declare_parameter('overlap_ratio', 0.15)            # 15% overlap for safety
        self.declare_parameter('robot_radius', 0.18)             # 18cm Main body radius
        # NOTE: Robot has a side brush (4cm radius) at front-left that extends reach.
        # This allows cleaning edges even with 18cm radius body.
        
        self.declare_parameter('timeout_per_waypoint', 90)       # Seconds per goal
        self.declare_parameter('start_on_exploration_complete', False)  # Wait for explicit start command
        self.declare_parameter('min_region_area', 0.02)          # Min area to cover (m²) - lowered for small spaces
        self.declare_parameter('max_retries', 1)                 # Number of times to retry missed areas
        
        # Direct movement parameters (turn-then-drive)
        self.declare_parameter('use_direct_drive', True)         # Use direct cmd_vel instead of Nav2
        self.declare_parameter('linear_speed', 0.12)             # Forward speed (m/s)
        self.declare_parameter('angular_speed', 0.4)             # Max turn speed (rad/s)
        self.declare_parameter('min_angular_speed', 0.15)        # Min turn speed (rad/s)
        self.declare_parameter('angle_tolerance', 0.35)          # Radians (~20°) - tolerance for turning
        self.declare_parameter('position_tolerance', 0.10)       # Meters (10cm)
        
        self.coverage_width = self.get_parameter('coverage_width').value
        self.overlap_ratio = self.get_parameter('overlap_ratio').value
        self.robot_radius = self.get_parameter('robot_radius').value
        self.timeout = self.get_parameter('timeout_per_waypoint').value
        self.auto_start = self.get_parameter('start_on_exploration_complete').value
        self.min_region_area = self.get_parameter('min_region_area').value
        self.max_retries = self.get_parameter('max_retries').value
        
        # Direct movement parameters
        self.use_direct_drive = self.get_parameter('use_direct_drive').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.min_angular_speed = self.get_parameter('min_angular_speed').value
        self.angle_tolerance = self.get_parameter('angle_tolerance').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        
        # Effective step between lines
        self.step_size = self.coverage_width * (1 - self.overlap_ratio)

        # ===================== Action Client (fallback) =====================
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ===================== Subscribers =====================
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, map_qos)
        
        self.exploration_complete_sub = self.create_subscription(
            Bool, 'exploration_complete', self.exploration_complete_callback, 10)
        
        # Control subscriber
        self.control_sub = self.create_subscription(
            String, 'coverage_control', self.control_callback, 10)

        # ===================== Publishers =====================
        self.coverage_complete_pub = self.create_publisher(Bool, 'coverage_complete', 10)
        self.coverage_path_pub = self.create_publisher(Path, 'coverage_path', 10)
        self.waypoint_markers_pub = self.create_publisher(MarkerArray, 'coverage_waypoints', 10)
        self.state_pub = self.create_publisher(String, 'coverage_state', 10)
        
        # Direct movement publisher (bypasses Nav2)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel_nav', 10)
        
        # Odometry subscriber for position feedback
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10)

        # ===================== State =====================
        self.coverage_state = CoverageState.IDLE
        self.map_array = None
        self.map_info = None
        self.inflated_map = None
        
        self.waypoints = []
        self.current_waypoint_idx = 0
        self.is_navigating = False
        self.mission_started = False
        self.mission_complete = False
        self.retry_count = 0
        self.missed_waypoints = []
        
        # For pause/resume
        self.current_goal_handle = None
        
        # Robot pose (from odometry)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.odom_received = False
        
        # Movement state for direct drive
        self.movement_phase = 'idle'  # 'idle', 'turning', 'driving'
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_yaw = 0.0
        self.last_turn_direction = 1  # 1 = counter-clockwise (left), -1 = clockwise (right)
        self.last_cmd_time = 0.0  # For rate limiting command publishing
        self.control_rate = 20.0  # Hz - how often to send cmd_vel commands
        self.drive_start_time = 0.0  # When driving phase started (for minimum drive time)
        self.min_drive_time = 0.5  # Minimum seconds to drive before allowing re-turn
        
        # Statistics
        self.successful_waypoints = 0
        self.failed_waypoints = 0
        self.start_time = None
        self.total_distance = 0.0
        self.last_status_time = 0.0  # For periodic status logging

        # ===================== Timer =====================
        self.create_timer(1.0, self.publish_state)
        self.create_timer(3.0, self.log_status)  # Log status every 3 seconds during operation

        self.get_logger().info('=' * 60)
        self.get_logger().info('🧹 Adaptive Coverage Planner Started')
        self.get_logger().info(f'   Coverage width: {self.coverage_width * 100:.0f}cm')
        self.get_logger().info(f'   Step size: {self.step_size * 100:.1f}cm')
        if self.use_direct_drive:
            self.get_logger().info(f'   Movement: DIRECT DRIVE (turn-then-straight)')
            self.get_logger().info(f'   Speed: {self.linear_speed} m/s, Turn: {self.angular_speed} rad/s')
        else:
            self.get_logger().info(f'   Movement: Nav2 (path planning)')
        self.get_logger().info('   Waiting for start command...')
        self.get_logger().info('=' * 60)

    def control_callback(self, msg: String):
        """Handle external control commands."""
        command = msg.data.lower().strip()
        self.get_logger().info(f'📬 Coverage received command: "{command}"')
        
        if command == 'start':
            self.handle_start()
        elif command == 'stop':
            self.handle_stop()
        elif command == 'pause':
            self.handle_pause()
        elif command == 'resume':
            self.handle_resume()
        elif command == 'reset':
            self.handle_reset()
        else:
            self.get_logger().warn(f'❓ Unknown command: "{command}"')

    def handle_start(self):
        """Start coverage."""
        if self.coverage_state == CoverageState.IDLE:
            self.get_logger().info('▶️ Starting coverage...')
            self.start_coverage_mission()
        elif self.coverage_state == CoverageState.STOPPED:
            # Resume from stopped - continue where we left off
            self.get_logger().info('▶️ Resuming coverage from stopped state...')
            self.coverage_state = CoverageState.RUNNING
            if self.waypoints and self.current_waypoint_idx < len(self.waypoints):
                self.send_next_goal()
            else:
                self.start_coverage_mission()
        elif self.coverage_state == CoverageState.PAUSED:
            self.handle_resume()
        elif self.coverage_state == CoverageState.COMPLETE:
            # Allow restarting from complete state
            self.get_logger().info('▶️ Restarting coverage from complete state...')
            self.handle_reset()
            self.start_coverage_mission()
        else:
            self.get_logger().info(f'   Already in state: {self.coverage_state}')

    def handle_stop(self):
        """Stop coverage."""
        if self.coverage_state in [CoverageState.RUNNING, CoverageState.PAUSED]:
            self.get_logger().info('🛑 Stopping coverage...')
            self.cancel_current_goal()
            self.coverage_state = CoverageState.STOPPED
            self.is_navigating = False
        else:
            self.get_logger().info(f'   Already in state: {self.coverage_state}')

    def handle_pause(self):
        """Pause coverage."""
        if self.coverage_state == CoverageState.RUNNING:
            self.get_logger().info('⏸️ Pausing coverage...')
            self.cancel_current_goal()
            self.coverage_state = CoverageState.PAUSED
            self.is_navigating = False
        else:
            self.get_logger().info(f'   Cannot pause from state: {self.coverage_state}')

    def handle_resume(self):
        """Resume coverage from pause."""
        if self.coverage_state == CoverageState.PAUSED:
            self.get_logger().info('▶️ Resuming coverage...')
            self.coverage_state = CoverageState.RUNNING
            self.send_next_goal()
        elif self.coverage_state == CoverageState.STOPPED:
            self.handle_start()
        else:
            self.get_logger().info(f'   Cannot resume from state: {self.coverage_state}')

    def handle_reset(self):
        """Reset coverage state."""
        self.get_logger().info('🔄 Resetting coverage...')
        self.cancel_current_goal()
        
        # Reset all state
        self.coverage_state = CoverageState.IDLE
        self.mission_started = False
        self.mission_complete = False
        self.is_navigating = False
        self.waypoints = []
        self.current_waypoint_idx = 0
        self.retry_count = 0
        self.missed_waypoints = []
        self.successful_waypoints = 0
        self.failed_waypoints = 0
        self.start_time = None
        self.movement_phase = 'idle'
        self.last_turn_direction = 1  # Reset to default (counter-clockwise)
        self.drive_start_time = 0.0

    def cancel_current_goal(self):
        """Cancel the current navigation goal and stop robot."""
        # Stop direct movement
        self.movement_phase = 'idle'
        self.stop_robot()
        
        # Cancel Nav2 goal if active
        if self.current_goal_handle is not None:
            self.get_logger().info('   🛑 Canceling current goal...')
            try:
                cancel_future = self.current_goal_handle.cancel_goal_async()
                # We don't wait for the result to avoid blocking
            except Exception as e:
                self.get_logger().warn(f'   Could not cancel goal: {e}')
            self.current_goal_handle = None

    def stop_robot(self):
        """Send zero velocity to stop the robot."""
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)

    def odom_callback(self, msg: Odometry):
        """Update robot pose from odometry."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.odom_received = True
        
        # If we're in direct drive mode and actively moving, run the control loop
        # Rate limit to avoid flooding cmd_vel
        if self.use_direct_drive and self.coverage_state == CoverageState.RUNNING:
            current_time = self.get_clock().now().nanoseconds / 1e9
            if current_time - self.last_cmd_time >= 1.0 / self.control_rate:
                self.last_cmd_time = current_time
                if self.movement_phase == 'turning':
                    self.execute_turn()
                elif self.movement_phase == 'driving':
                    self.execute_drive()

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def execute_turn(self):
        """
        NEW APPROACH: Always drive forward with steering correction.
        Only do pure rotation if pointing completely wrong direction (>90°).
        """
        # Safety check
        if self.movement_phase != 'turning':
            return
            
        # Calculate to target
        dx = self.target_x - self.robot_x
        dy = self.target_y - self.robot_y
        distance_to_target = math.sqrt(dx*dx + dy*dy)
        
        # If we're already very close to target, move to next
        if distance_to_target < self.position_tolerance:
            self.get_logger().info(f'   ✅ Already at target')
            self.movement_phase = 'idle'
            self.successful_waypoints += 1
            self.current_waypoint_idx += 1
            self.is_navigating = False
            self.send_next_goal()
            return
        
        target_angle = math.atan2(dy, dx)
        angle_error = self.normalize_angle(target_angle - self.robot_yaw)
        
        self.get_logger().info(f'   🎯 Target: ({self.target_x:.2f}, {self.target_y:.2f}), '
                               f'Robot: ({self.robot_x:.2f}, {self.robot_y:.2f}), '
                               f'Yaw: {math.degrees(self.robot_yaw):.0f}°, '
                               f'Target angle: {math.degrees(target_angle):.0f}°, '
                               f'Error: {math.degrees(angle_error):.0f}°')
        
        # If angle error is HUGE (>90°), do pure rotation first
        if abs(angle_error) > 1.57:  # >90 degrees
            cmd = Twist()
            cmd.linear.x = 0.0
            if angle_error > 0:
                cmd.angular.z = self.angular_speed
            else:
                cmd.angular.z = -self.angular_speed
            self.cmd_vel_pub.publish(cmd)
            self.get_logger().info(f'   🔄 Big turn needed: {math.degrees(angle_error):.0f}°')
            return
        
        # Otherwise: DRIVE FORWARD with steering correction!
        # This is much more robust than turn-then-drive
        self.get_logger().info(f'   ➡️ Driving with correction...')
        self.movement_phase = 'driving'
        self.drive_start_time = self.get_clock().now().nanoseconds / 1e9
        
        cmd = Twist()
        cmd.linear.x = self.linear_speed
        # Proportional steering - stronger correction for bigger errors
        cmd.angular.z = angle_error * 0.5  # P-controller for steering
        # Limit steering
        cmd.angular.z = max(-self.angular_speed, min(self.angular_speed, cmd.angular.z))
        self.cmd_vel_pub.publish(cmd)

    def execute_drive(self):
        """Execute driving phase - drive toward target with steering."""
        # Safety check
        if self.movement_phase != 'driving':
            return
            
        dx = self.target_x - self.robot_x
        dy = self.target_y - self.robot_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance < self.position_tolerance:
            # Waypoint reached!
            self.stop_robot()
            self.get_logger().info(f'   ✅ Reached waypoint (dist={distance:.3f}m)')
            self.movement_phase = 'idle'
            self.successful_waypoints += 1
            self.current_waypoint_idx += 1
            self.is_navigating = False
            
            # Move to next waypoint
            self.send_next_goal()
            return
        
        # Calculate steering
        target_angle = math.atan2(dy, dx)
        angle_error = self.normalize_angle(target_angle - self.robot_yaw)
        
        # If we're pointing completely wrong way, go back to turning
        if abs(angle_error) > 1.57:  # >90 degrees
            self.get_logger().info(f'   ↩️ Way off course ({math.degrees(angle_error):.0f}°), need to turn')
            self.movement_phase = 'turning'
            return
        
        # ALWAYS drive forward with proportional steering
        cmd = Twist()
        cmd.linear.x = self.linear_speed
        
        # Steering: proportional to angle error
        cmd.angular.z = angle_error * 0.5
        # Limit steering speed
        cmd.angular.z = max(-self.angular_speed, min(self.angular_speed, cmd.angular.z))
        
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().debug(f'   🚗 Drive: dist={distance:.2f}m, steer={math.degrees(angle_error):.0f}°')

    def publish_state(self):
        """Publish current coverage state."""
        msg = String()
        msg.data = self.coverage_state
        self.state_pub.publish(msg)

    def log_status(self):
        """Log periodic status during operation - useful for debugging."""
        if self.coverage_state != CoverageState.RUNNING:
            return
        
        # Calculate distance to current target
        if self.waypoints and self.current_waypoint_idx < len(self.waypoints):
            dx = self.target_x - self.robot_x
            dy = self.target_y - self.robot_y
            dist_to_target = math.sqrt(dx*dx + dy*dy)
            target_angle = math.atan2(dy, dx)
            angle_error = self.normalize_angle(target_angle - self.robot_yaw)
            
            progress = (self.current_waypoint_idx + 1) / len(self.waypoints) * 100
            
            self.get_logger().info(
                f'📊 Status: {self.movement_phase.upper()} | '
                f'WP {self.current_waypoint_idx + 1}/{len(self.waypoints)} ({progress:.0f}%) | '
                f'Pos: ({self.robot_x:.2f}, {self.robot_y:.2f}) | '
                f'Target: ({self.target_x:.2f}, {self.target_y:.2f}) | '
                f'Dist: {dist_to_target:.2f}m | '
                f'Heading err: {math.degrees(angle_error):.0f}°'
            )

    def map_callback(self, msg: OccupancyGrid):
        """Store latest map."""
        self.map_info = msg.info
        self.map_array = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width))

    def exploration_complete_callback(self, msg: Bool):
        """Triggered when exploration finishes."""
        if msg.data and self.auto_start and not self.mission_started:
            self.get_logger().info('📬 Exploration complete signal received!')
            self.start_coverage_mission()

    def start_coverage_mission(self):
        """Generate coverage path and start executing it."""
        # Wait for map with timeout and retry
        max_wait_time = 10.0  # seconds
        wait_interval = 0.5
        waited = 0.0
        
        while self.map_array is None and waited < max_wait_time:
            self.get_logger().info(f'⏳ Waiting for map... ({waited:.1f}s)')
            import time
            time.sleep(wait_interval)
            waited += wait_interval
            # Spin to process callbacks
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.map_array is None:
            self.get_logger().error('❌ No map available after waiting! Make sure SLAM is running.')
            self.get_logger().error('   Try running /scan first to build a map.')
            return
        
        # Check map quality
        height, width = self.map_array.shape
        free_count = np.sum((self.map_array >= 0) & (self.map_array < FREE_THRESHOLD))
        unknown_count = np.sum(self.map_array == -1)
        occupied_count = np.sum(self.map_array >= OCCUPIED_THRESHOLD)
        
        self.get_logger().info(f'📊 Map stats: {width}x{height}, free={free_count}, unknown={unknown_count}, occupied={occupied_count}')
        
        if free_count < 50:
            self.get_logger().error('❌ Map has very little free space! Run exploration first.')
            return
        
        if self.mission_started and self.coverage_state == CoverageState.RUNNING:
            self.get_logger().warn('⚠️ Mission already in progress')
            return

        self.get_logger().info('')
        self.get_logger().info('🔧 Generating coverage path...')
        
        # Step 1: Inflate obstacles for robot safety
        self.inflate_obstacles()
        
        # Step 2: Generate adaptive coverage path
        self.waypoints = self.generate_adaptive_coverage_path()
        
        # Fallback: If complex path generation failed, try simple zigzag
        if not self.waypoints:
            self.get_logger().warn('⚠️ Complex path generation failed, trying simple zigzag...')
            self.waypoints = self.generate_simple_zigzag_path()
        
        if not self.waypoints:
            self.get_logger().error('❌ Could not generate coverage path!')
            self.get_logger().error('   Check that the map has sufficient free space.')
            return
        
        # Step 3: Publish path for visualization
        self.publish_coverage_path()
        self.publish_waypoint_markers()
        
        # Step 4: Start mission
        self.mission_started = True
        self.coverage_state = CoverageState.RUNNING
        self.start_time = self.get_clock().now()
        self.current_waypoint_idx = 0
        
        self.get_logger().info(f'✅ Generated {len(self.waypoints)} waypoints')
        self.get_logger().info('')
        self.get_logger().info('🚀 Starting coverage mission!')
        self.get_logger().info('-' * 60)
        
        self.send_next_goal()

    def inflate_obstacles(self):
        """Inflate obstacles by robot radius for safe navigation."""
        # Create obstacle mask
        obstacle_mask = self.map_array >= OCCUPIED_THRESHOLD
        
        # Calculate inflation in cells
        inflation_cells = int(self.robot_radius / self.map_info.resolution) + 1
        
        # Create circular kernel
        kernel_size = 2 * inflation_cells + 1
        y, x = np.ogrid[-inflation_cells:inflation_cells+1, -inflation_cells:inflation_cells+1]
        kernel = (x*x + y*y <= inflation_cells*inflation_cells).astype(np.uint8)
        
        # Dilate obstacles
        inflated_obstacles = ndimage.binary_dilation(obstacle_mask, kernel)
        
        # Create inflated map
        self.inflated_map = self.map_array.copy()
        self.inflated_map[inflated_obstacles] = 100  # Mark as occupied

    def generate_adaptive_coverage_path(self) -> list:
        """
        Generate coverage path using Boustrophedon Cellular Decomposition.
        Dividing map into "Cells" (continuous regions) and cleaning each cell fully
        before moving to the next.
        """
        self.get_logger().info('Build Cells (Boustrophedon decomposition)...')
        
        height, width = self.inflated_map.shape
        resolution = self.map_info.resolution
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y
        
        self.get_logger().info(f'Map size: {width}x{height} cells, resolution: {resolution}m')
        self.get_logger().info(f'Map origin: ({origin_x}, {origin_y})')
        
        step_cells = max(1, int(self.step_size / resolution))
        self.get_logger().info(f'Step size: {self.step_size}m = {step_cells} cells')
        
        # Count free cells for debugging
        free_count = np.sum((self.inflated_map >= 0) & (self.inflated_map < FREE_THRESHOLD))
        self.get_logger().info(f'Free cells in inflated map: {free_count} ({100*free_count/(height*width):.1f}%)')
        
        if free_count < 10:
            self.get_logger().error('Not enough free space in map!')
            return []
        
        # --- 1. Decompose Space into Cells ---
        # Cells structure: key=cell_id, value={'segments': [], 'connections': set(), 'center': (x,y)}
        cells = {}
        active_cell_indices = {} # Map from segment_index in *previous column* to cell_id
        
        # To track connectivity
        prev_segments = []
        cell_counter = 0
        total_segments_found = 0

        for col in range(0, width, step_cells):
            curr_segments_raw = self.find_free_segments_in_column(col)
            total_segments_found += len(curr_segments_raw)
            # Add x, y_start, y_end info
            curr_segments = []
            x = origin_x + (col + 0.5) * resolution
            
            for seg in curr_segments_raw:
                y_start = origin_y + (seg[0] + 0.5) * resolution
                y_end = origin_y + (seg[1] + 0.5) * resolution
                curr_segments.append({
                    'raw': seg,
                    'vals': (x, y_start, y_end, col, seg[0], seg[1])
                })
            
            # Match current segments with previous segments to find overlapping cells
            curr_to_cell_map = {} # Map curr_seg_idx -> cell_id
            
            # Build overlap matrix
            # connections[curr_idx] = [prev_idx, prev_idx...]
            overlap_map = [[] for _ in curr_segments]
            
            for i, c_seg in enumerate(curr_segments):
                c_y1, c_y2 = c_seg['vals'][1], c_seg['vals'][2]
                
                for j, p_seg in enumerate(prev_segments):
                    p_y1, p_y2 = p_seg['vals'][1], p_seg['vals'][2]
                    
                    # Check overlap (with slight tolerance)
                    if max(c_y1, p_y1) < min(c_y2, p_y2) + resolution:
                        overlap_map[i].append(j)

            # Assign cells based on overlaps
            for i, overlaps in enumerate(overlap_map):
                if len(overlaps) == 0:
                    # Case: New Cell (Start)
                    cell_counter += 1
                    cid = cell_counter
                    cells[cid] = {'segments': [], 'connections': set(), 'visited': False}
                    curr_to_cell_map[i] = cid
                    
                elif len(overlaps) == 1:
                    # Case: Continue Cell
                    prev_idx = overlaps[0]
                    # Check if this prev segment split into multiple current (Split event)
                    # Count how many currents map to this prev
                    count_splits = sum(1 for o_list in overlap_map if prev_idx in o_list)
                    
                    cid = active_cell_indices[prev_idx]
                    
                    if count_splits > 1:
                        # Split event! Prev cell ends. New cells start.
                        cell_counter += 1
                        new_cid = cell_counter
                        cells[new_cid] = {'segments': [], 'connections': {cid}, 'visited': False}
                        cells[cid]['connections'].add(new_cid)
                        curr_to_cell_map[i] = new_cid
                    else:
                        # Standard continuation
                        curr_to_cell_map[i] = cid

                else:
                    # Case: Merge (Multiple prev segments -> 1 curr segment)
                    # Identify all prev cells
                    prev_cids = {active_cell_indices[p_idx] for p_idx in overlaps}
                    
                    # Merge event: All prev cells end. New cell starts.
                    cell_counter += 1
                    new_cid = cell_counter
                    cells[new_cid] = {'segments': [], 'connections': set(), 'visited': False}
                    
                    # Connect all merging cells to this new one
                    for pcid in prev_cids:
                        cells[pcid]['connections'].add(new_cid)
                        cells[new_cid]['connections'].add(pcid)
                        
                    curr_to_cell_map[i] = new_cid

            # Store segments in cells
            for i, c_seg in enumerate(curr_segments):
                if i in curr_to_cell_map:
                    cid = curr_to_cell_map[i]
                    cells[cid]['segments'].append(c_seg['vals'])
                else:
                    # Orphan segment - create new cell for it
                    cell_counter += 1
                    cid = cell_counter
                    cells[cid] = {'segments': [c_seg['vals']], 'connections': set(), 'visited': False}
                    curr_to_cell_map[i] = cid
            
            # Prepare for next iteration
            prev_segments = curr_segments
            active_cell_indices = curr_to_cell_map

        # Debug: Check for empty cells
        empty_cells = [cid for cid, data in cells.items() if not data['segments']]
        if empty_cells:
            self.get_logger().warn(f'Found {len(empty_cells)} cells with no segments')
            # Remove empty cells
            for cid in empty_cells:
                del cells[cid]

        self.get_logger().info(f'Total segments found across all columns: {total_segments_found}')
        self.get_logger().info(f'Detected {len(cells)} distinct cells/regions.')
        
        if not cells:
            self.get_logger().error('No cells were created! Check that map has navigable free space.')
            return []
        
        # --- 2. Filter Tiny Cells ---
        valid_cells = {}
        for cid, data in cells.items():
            # Calculate actual area: sum of (segment_height * step_size) for each segment
            # Each segment is (x, y_start, y_end, col, r1, r2)
            cell_area = 0.0
            for seg in data['segments']:
                segment_height = abs(seg[2] - seg[1])  # y_end - y_start
                cell_area += segment_height * self.step_size
            
            if cell_area > self.min_region_area:
                valid_cells[cid] = data
                self.get_logger().debug(f'   Cell {cid}: area={cell_area:.3f}m², segments={len(data["segments"])}')
            else:
                self.get_logger().debug(f'   Cell {cid}: FILTERED (area={cell_area:.3f}m² < {self.min_region_area}m²)')
        
        self.get_logger().info(f'After filtering: {len(valid_cells)} valid cells (min area: {self.min_region_area}m²)')
        
        cells = valid_cells
        if not cells:
            self.get_logger().error('All cells were filtered out! Try reducing min_region_area parameter.')
            return []

        # --- 3. Order Cells (TSP / Nearest Neighbor) ---
        ordered_cell_ids = []
        # Find cell closest to (0,0) or start
        
        # Calculate centroids
        for cid, data in cells.items():
            xs = [s[0] for s in data['segments']]
            ys = [s[1] for s in data['segments']] # use starty approx
            data['centroid'] = (sum(xs)/len(xs), sum(ys)/len(ys))
        
        # Start with closest to robot (assumed 0,0 for now or first waypoints)
        curr_pos = (0.0, 0.0)
        
        remaining_ids = set(cells.keys())
        
        while remaining_ids:
            # Find nearest cell to current position
            best_id = None
            min_dist = float('inf')
            
            for cid in remaining_ids:
                cx, cy = cells[cid]['centroid']
                dist = (cx - curr_pos[0])**2 + (cy - curr_pos[1])**2
                
                # Biased search: prefer connected neighbors of last visited cell
                # (Not implemented fully here, but implicit by distance usually works)
                
                if dist < min_dist:
                    min_dist = dist
                    best_id = cid
            
            ordered_cell_ids.append(best_id)
            remaining_ids.remove(best_id)
            curr_pos = cells[best_id]['centroid']

        # --- 4. Generate Paths per Cell (Zigzag only - simpler and more reliable) ---
        final_waypoints = []
        
        self.get_logger().info(f'Generating waypoints for {len(ordered_cell_ids)} cells...')
        
        for cid in ordered_cell_ids:
            cell_segments = cells[cid]['segments']
            
            if not cell_segments:
                self.get_logger().warn(f'Cell {cid} has no segments, skipping')
                continue
            
            # Sort segments by x coordinate to ensure proper order
            cell_segments = sorted(cell_segments, key=lambda s: s[0])
            
            # Generate Zigzag pattern - alternating up/down passes
            going_up = True
            
            for x, y_start, y_end, col, r1, r2 in cell_segments:
                # Skip very short segments
                if abs(y_end - y_start) < 0.05:  # Less than 5cm
                    continue
                    
                if going_up:
                    final_waypoints.append((x, y_start, math.pi / 2))   # Face up
                    final_waypoints.append((x, y_end, math.pi / 2))     # Face up
                else:
                    final_waypoints.append((x, y_end, -math.pi / 2))    # Face down
                    final_waypoints.append((x, y_start, -math.pi / 2))  # Face down
                going_up = not going_up
        
        self.get_logger().info(f'Generated {len(final_waypoints)} total waypoints')
        
        if not final_waypoints:
            self.get_logger().error('No waypoints generated! Check map quality and parameters.')
            
        return final_waypoints

    def generate_cell_boundary(self, segments) -> list:
        """
        Generate a Clockwise path around the given cell segments.
        This puts the left side of the robot (side brush) against the wall/boundary.
        """
        waypoints = []
        if not segments:
            return []
            
        # Segments are sorted by column (x)
        # Structure: (x, y_start, y_end, col, r1, r2)
        
        # 1. Left Edge (first column) - Go Up
        first_seg = segments[0]
        waypoints.append((first_seg[0], first_seg[1], math.pi/2)) # Start of col
        waypoints.append((first_seg[0], first_seg[2], math.pi/2)) # End of col
        
        # 2. Top Edge (scan left to right)
        # Connect y_end of col i to y_end of col i+1
        for i in range(len(segments) - 1):
            curr_s = segments[i]
            next_s = segments[i+1]
            # Waypoint at current top -> next top
            # Orientation: Pointing Right (0) or towards next point
            yaw = math.atan2(next_s[2] - curr_s[2], next_s[0] - curr_s[0])
            waypoints.append((curr_s[0], curr_s[2], yaw))
            waypoints.append((next_s[0], next_s[2], yaw))
            
        # 3. Right Edge (last column) - Go Down
        last_seg = segments[-1]
        waypoints.append((last_seg[0], last_seg[2], -math.pi/2)) # Top of last col
        waypoints.append((last_seg[0], last_seg[1], -math.pi/2)) # Bottom of last col
        
        # 4. Bottom Edge (scan right to left)
        # Connect y_start of col i to y_start of col i-1
        for i in range(len(segments) - 1, 0, -1):
            curr_s = segments[i]
            prev_s = segments[i-1]
            # Waypoint at current bottom -> prev bottom
            yaw = math.atan2(prev_s[1] - curr_s[1], prev_s[0] - curr_s[0])
            waypoints.append((curr_s[0], curr_s[1], yaw))
            waypoints.append((prev_s[0], prev_s[1], yaw))
            
        return waypoints


    def find_free_segments_in_column(self, col: int) -> list:
        """
        Find continuous free segments in a column.
        Returns list of (start_row, end_row) tuples.
        """
        segments = []
        height = self.inflated_map.shape[0]
        
        in_segment = False
        seg_start = 0
        
        for row in range(height):
            cell_value = self.inflated_map[row, col]
            is_free = 0 <= cell_value < FREE_THRESHOLD
            
            if is_free and not in_segment:
                # Start new segment
                in_segment = True
                seg_start = row
            elif not is_free and in_segment:
                # End segment
                in_segment = False
                seg_end = row - 1
                
                # Only add if segment is long enough (at least 10cm)
                segment_length = (seg_end - seg_start) * self.map_info.resolution
                if segment_length >= 0.10:  # At least 10cm
                    segments.append((seg_start, seg_end))
        
        # Handle segment that goes to end of column
        if in_segment:
            seg_end = height - 1
            segment_length = (seg_end - seg_start) * self.map_info.resolution
            if segment_length >= 0.10:  # At least 10cm
                segments.append((seg_start, seg_end))
        
        return segments

    def generate_simple_zigzag_path(self) -> list:
        """
        Fallback: Generate a simple zigzag coverage path over the entire free space.
        This is simpler and more robust than cell decomposition.
        """
        self.get_logger().info('Generating simple zigzag path...')
        
        height, width = self.inflated_map.shape
        resolution = self.map_info.resolution
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y
        
        step_cells = max(1, int(self.step_size / resolution))
        waypoints = []
        
        going_up = True
        
        # Scan through columns at step_size intervals
        for col in range(0, width, step_cells):
            # Find all free segments in this column
            segments = self.find_free_segments_in_column(col)
            
            if not segments:
                continue
            
            x = origin_x + (col + 0.5) * resolution
            
            # Process segments in order (or reverse order if going down)
            if not going_up:
                segments = segments[::-1]
            
            for seg_start, seg_end in segments:
                y_start = origin_y + (seg_start + 0.5) * resolution
                y_end = origin_y + (seg_end + 0.5) * resolution
                
                # Skip very short segments
                if abs(y_end - y_start) < 0.05:
                    continue
                
                if going_up:
                    waypoints.append((x, y_start, math.pi / 2))   # Face up
                    waypoints.append((x, y_end, math.pi / 2))     # Face up
                else:
                    waypoints.append((x, y_end, -math.pi / 2))    # Face down
                    waypoints.append((x, y_start, -math.pi / 2))  # Face down
            
            going_up = not going_up
        
        self.get_logger().info(f'Simple zigzag generated {len(waypoints)} waypoints')
        return waypoints

    def optimize_path_order(self, waypoints: list) -> list:
        """
        Reorder waypoint pairs to minimize travel distance.
        Treats waypoints as pairs (start and end of segments) and optimizes pair order.
        """
        if len(waypoints) <= 4:  # At least 2 pairs
            return waypoints
        
        # Group waypoints into pairs
        pairs = []
        for i in range(0, len(waypoints), 2):
            if i + 1 < len(waypoints):
                pairs.append((waypoints[i], waypoints[i+1]))
        
        # Start from first pair
        ordered_pairs = [pairs[0]]
        remaining_pairs = list(pairs[1:])
        
        while remaining_pairs:
            last_pair = ordered_pairs[-1]
            last_point = last_pair[1]  # End of last pair
            
            # Find nearest remaining pair
            min_dist = float('inf')
            nearest_idx = 0
            
            for i, pair in enumerate(remaining_pairs):
                # Distance to start of pair
                dist_to_start = math.sqrt((pair[0][0] - last_point[0])**2 + (pair[0][1] - last_point[1])**2)
                # Distance to end of pair (in case we reverse)
                dist_to_end = math.sqrt((pair[1][0] - last_point[0])**2 + (pair[1][1] - last_point[1])**2)
                
                dist = min(dist_to_start, dist_to_end)
                if dist < min_dist:
                    min_dist = dist
                    nearest_idx = i
            
            # Add the nearest pair, possibly reversed if closer to end
            nearest_pair = remaining_pairs[nearest_idx]
            dist_to_start = math.sqrt((nearest_pair[0][0] - last_point[0])**2 + (nearest_pair[0][1] - last_point[1])**2)
            dist_to_end = math.sqrt((nearest_pair[1][0] - last_point[0])**2 + (nearest_pair[1][1] - last_point[1])**2)
            
            if dist_to_end < dist_to_start:
                # Reverse the pair
                ordered_pairs.append((nearest_pair[1], nearest_pair[0]))
            else:
                ordered_pairs.append(nearest_pair)
            
            remaining_pairs.pop(nearest_idx)
        
        # Flatten back to list of waypoints
        ordered_waypoints = []
        for pair in ordered_pairs:
            ordered_waypoints.extend(pair)
        
        return ordered_waypoints

    def send_next_goal(self):
        """Send next waypoint - uses direct drive (turn-then-straight) or Nav2."""
        # Check if we should continue
        if self.coverage_state != CoverageState.RUNNING:
            self.get_logger().warn(f'   send_next_goal called but state is {self.coverage_state}')
            return
            
        if self.current_waypoint_idx >= len(self.waypoints):
            self.finish_mission()
            return

        x, y, yaw = self.waypoints[self.current_waypoint_idx]
        
        progress = (self.current_waypoint_idx + 1) / len(self.waypoints) * 100
        self.get_logger().info(
            f'[{self.current_waypoint_idx + 1}/{len(self.waypoints)}] '
            f'({progress:.0f}%) → ({x:.2f}, {y:.2f})')

        self.is_navigating = True
        
        if self.use_direct_drive:
            # Direct drive mode: Turn to face target, then drive straight
            self.target_x = x
            self.target_y = y
            self.target_yaw = yaw
            
            if not self.odom_received:
                self.get_logger().warn('⚠️ No odometry received yet, waiting...')
                # Create a one-shot timer to retry
                self.create_timer(0.5, self._retry_send_goal_once)
                return
            
            # Start the turn phase
            self.get_logger().info(f'   🔄 Turning to face target...')
            self.movement_phase = 'turning'
            
            # IMMEDIATELY send first turn command to start moving!
            self.execute_turn()
        else:
            # Nav2 mode (fallback)
            self.send_goal_nav2(x, y, yaw)

    def _retry_send_goal_once(self):
        """Retry sending goal after waiting for odom."""
        if self.odom_received and self.coverage_state == CoverageState.RUNNING:
            self.send_next_goal()

    def send_goal_nav2(self, x, y, yaw):
        """Send goal using Nav2 NavigateToPose action."""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('❌ Nav2 action server not available!')
            return
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection."""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().warn('   ⚠️ Goal rejected - skipping')
            self.failed_waypoints += 1
            # Add to missed list for later retry
            if self.current_waypoint_idx < len(self.waypoints):
                self.missed_waypoints.append(self.waypoints[self.current_waypoint_idx])
            
            self.is_navigating = False
            self.current_waypoint_idx += 1
            self.send_next_goal()
            return

        # Store handle for potential cancellation
        self.current_goal_handle = goal_handle
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle navigation result."""
        result = future.result()
        status = result.status
        
        self.is_navigating = False
        self.current_goal_handle = None
        
        # Check if we were stopped/paused
        if self.coverage_state != CoverageState.RUNNING:
            return
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info('   ✅')
            self.successful_waypoints += 1
        elif status == 6:  # ABORTED
            self.get_logger().warn('   ❌ (obstacle)')
            self.failed_waypoints += 1
            if self.current_waypoint_idx < len(self.waypoints):
                self.missed_waypoints.append(self.waypoints[self.current_waypoint_idx])
        elif status == 5:  # CANCELED
            self.get_logger().info('   🛑 (canceled)')
            # Don't count as failed, we'll resume later
            return
        else:
            self.get_logger().warn(f'   ❓ status={status}')
            self.failed_waypoints += 1
            if self.current_waypoint_idx < len(self.waypoints):
                self.missed_waypoints.append(self.waypoints[self.current_waypoint_idx])
        
        self.current_waypoint_idx += 1
        self.send_next_goal()

    def publish_coverage_path(self):
        """Publish coverage path for RViz visualization."""
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for x, y, yaw in self.waypoints:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            path_msg.poses.append(pose)
        
        self.coverage_path_pub.publish(path_msg)

    def publish_waypoint_markers(self):
        """Publish waypoint markers for RViz."""
        marker_array = MarkerArray()
        
        for i, (x, y, yaw) in enumerate(self.waypoints):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'coverage_waypoints'
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.05
            marker.pose.orientation.z = math.sin(yaw / 2.0)
            marker.pose.orientation.w = math.cos(yaw / 2.0)
            marker.scale.x = 0.1
            marker.scale.y = 0.02
            marker.scale.z = 0.02
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.7
            marker_array.markers.append(marker)
        
        self.waypoint_markers_pub.publish(marker_array)

    def finish_mission(self):
        """Called when coverage is complete."""
        
        # Check if we should retry missed spots
        if self.missed_waypoints and self.retry_count < self.max_retries:
            self.retry_count += 1
            self.get_logger().info('')
            self.get_logger().info('=' * 60)
            self.get_logger().info(f'⚠️ Detected {len(self.missed_waypoints)} uncleaned areas!')
            self.get_logger().info(f'🔄 Starting Retry Pass #{self.retry_count} to clean them...')
            self.get_logger().info('=' * 60)
            
            # Use missed waypoints as the new plan
            self.waypoints = self.missed_waypoints
            self.missed_waypoints = []
            
            # Optimize the visit order to minimize travel
            if len(self.waypoints) > 2:
                self.waypoints = self.optimize_path_order(self.waypoints)
            
            # Reset state for retry
            self.current_waypoint_idx = 0
            
            # Update visualization
            self.publish_coverage_path()
            self.publish_waypoint_markers()
            
            # Start retry mission
            self.send_next_goal()
            return

        self.mission_complete = True
        self.coverage_state = CoverageState.COMPLETE
        
        elapsed = 0
        if self.start_time:
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        total = self.successful_waypoints + self.failed_waypoints
        coverage_pct = (self.successful_waypoints / total * 100) if total > 0 else 0
        
        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info('🏁 COVERAGE MISSION COMPLETE!')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'   ⏱️ Time: {elapsed / 60:.1f} minutes')
        self.get_logger().info(f'   ✅ Successful: {self.successful_waypoints}/{total}')
        self.get_logger().info(f'   ❌ Failed: {self.failed_waypoints}/{total}')
        self.get_logger().info(f'   📊 Coverage: {coverage_pct:.0f}%')
        self.get_logger().info('=' * 60)
        
        # Publish completion
        msg = Bool()
        msg.data = True
        self.coverage_complete_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveCoveragePlanner()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Coverage interrupted by user')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
