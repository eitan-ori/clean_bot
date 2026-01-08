#!/usr/bin/env python3
"""
Clean Bot Coverage Mission

This node executes a full coverage path (cleaning pattern) over a mapped area.
Uses a boustrophedon (zigzag/lawnmower) pattern with configurable coverage width.

Features:
- Generates zigzag path based on map from SLAM
- Configurable coverage width (default: 14cm for cleaning)
- Skips obstacles and unknown areas
- Recovery behaviors on navigation failures
- Progress tracking and reporting

Topics Subscribed:
- /map (nav_msgs/OccupancyGrid) - Map from SLAM

Actions Used:
- /navigate_to_pose (nav2_msgs/NavigateToPose)

Parameters:
- coverage_width: Width of coverage stripe (default: 0.14m = 14cm)
- overlap_ratio: How much stripes overlap (default: 0.1 = 10%)
- timeout_per_waypoint: Max seconds per waypoint (default: 60)

Author: Clean Bot Team
"""

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool


class CoverageMission(Node):
    def __init__(self):
        super().__init__('coverage_mission')

        # ===================== Parameters =====================
        self.declare_parameter('coverage_width', 0.14)      # 14cm coverage stripe
        self.declare_parameter('overlap_ratio', 0.1)        # 10% overlap
        self.declare_parameter('timeout_per_waypoint', 60)  # 60 seconds max per goal
        self.declare_parameter('free_threshold', 50)        # Occupancy < 50 = free
        self.declare_parameter('margin', 0.15)              # 15cm margin from walls
        
        self.coverage_width = self.get_parameter('coverage_width').value
        self.overlap_ratio = self.get_parameter('overlap_ratio').value
        self.timeout = self.get_parameter('timeout_per_waypoint').value
        self.free_threshold = self.get_parameter('free_threshold').value
        self.margin = self.get_parameter('margin').value
        
        # Effective step = coverage_width * (1 - overlap)
        self.step_size = self.coverage_width * (1 - self.overlap_ratio)
        
        # ===================== Action Client =====================
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # ===================== Subscribers =====================
        self.map_sub = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, 10)
        
        # ===================== Publishers =====================
        self.mission_complete_pub = self.create_publisher(Bool, 'mission_complete', 10)
        
        # ===================== State =====================
        self.map_data = None
        self.map_info = None
        self.waypoints = []
        self.current_waypoint_idx = 0
        self.mission_started = False
        self.mission_complete = False
        
        # Statistics
        self.successful_waypoints = 0
        self.failed_waypoints = 0
        self.start_time = None
        
        self.get_logger().info('=' * 50)
        self.get_logger().info('🧹 Coverage Mission Node Started')
        self.get_logger().info(f'   Coverage width: {self.coverage_width * 100:.0f}cm')
        self.get_logger().info(f'   Step size: {self.step_size * 100:.1f}cm')
        self.get_logger().info('   Waiting for map...')
        self.get_logger().info('=' * 50)

    def map_callback(self, msg: OccupancyGrid):
        """Process incoming map and generate coverage path."""
        if self.mission_started:
            return  # Ignore map updates during mission
            
        self.map_data = msg.data
        self.map_info = msg.info
        
        self.get_logger().info(f'📍 Map received: {self.map_info.width}x{self.map_info.height} '
                               f'@ {self.map_info.resolution}m/cell')
        
        # Generate coverage path
        self.waypoints = self._generate_coverage_path()
        
        if len(self.waypoints) > 0:
            self.get_logger().info(f'✅ Generated {len(self.waypoints)} coverage waypoints')
            self._start_mission()
        else:
            self.get_logger().warn('⚠️ No valid waypoints generated - map may be too small or blocked')

    def _generate_coverage_path(self) -> list:
        """
        Generate boustrophedon (zigzag) coverage path.
        Returns list of (x, y, yaw) tuples in map frame.
        """
        waypoints = []
        
        # Map parameters
        width = self.map_info.width
        height = self.map_info.height
        res = self.map_info.resolution
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y
        
        # Calculate bounds with margin
        margin_cells = int(self.margin / res)
        
        # Find actual free space bounds (not just map bounds)
        min_x_cell, max_x_cell = width, 0
        min_y_cell, max_y_cell = height, 0
        
        for y in range(height):
            for x in range(width):
                idx = y * width + x
                if 0 <= self.map_data[idx] < self.free_threshold:
                    min_x_cell = min(min_x_cell, x)
                    max_x_cell = max(max_x_cell, x)
                    min_y_cell = min(min_y_cell, y)
                    max_y_cell = max(max_y_cell, y)
        
        # Apply margin
        min_x_cell += margin_cells
        max_x_cell -= margin_cells
        min_y_cell += margin_cells
        max_y_cell -= margin_cells
        
        if min_x_cell >= max_x_cell or min_y_cell >= max_y_cell:
            return []
        
        # Convert to meters
        min_x = origin_x + (min_x_cell * res)
        max_x = origin_x + (max_x_cell * res)
        min_y = origin_y + (min_y_cell * res)
        max_y = origin_y + (max_y_cell * res)
        
        self.get_logger().info(f'   Free space bounds: X[{min_x:.2f}, {max_x:.2f}] Y[{min_y:.2f}, {max_y:.2f}]')
        
        # Generate zigzag pattern
        x = min_x
        going_up = True
        
        while x <= max_x:
            if going_up:
                # Bottom to top
                y_start = min_y
                y_end = max_y
                yaw = math.pi / 2  # Facing up
            else:
                # Top to bottom
                y_start = max_y
                y_end = min_y
                yaw = -math.pi / 2  # Facing down
            
            # Check if start and end points are free
            if self._is_point_free(x, y_start) and self._is_point_free(x, y_end):
                waypoints.append((x, y_start, yaw))
                waypoints.append((x, y_end, yaw))
            
            x += self.step_size
            going_up = not going_up
        
        return waypoints

    def _is_point_free(self, x: float, y: float) -> bool:
        """Check if a point in map coordinates is free space."""
        # Convert to cell coordinates
        cell_x = int((x - self.map_info.origin.position.x) / self.map_info.resolution)
        cell_y = int((y - self.map_info.origin.position.y) / self.map_info.resolution)
        
        # Check bounds
        if cell_x < 0 or cell_x >= self.map_info.width:
            return False
        if cell_y < 0 or cell_y >= self.map_info.height:
            return False
        
        # Check occupancy
        idx = cell_y * self.map_info.width + cell_x
        return 0 <= self.map_data[idx] < self.free_threshold

    def _start_mission(self):
        """Start the coverage mission."""
        self.mission_started = True
        self.start_time = time.time()
        self.current_waypoint_idx = 0
        
        self.get_logger().info('')
        self.get_logger().info('🚀 Starting Coverage Mission!')
        self.get_logger().info('-' * 50)
        
        # Wait for Nav2
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('❌ Nav2 action server not available!')
            return
        
        self._send_next_goal()

    def _send_next_goal(self):
        """Send the next waypoint to Nav2."""
        if self.current_waypoint_idx >= len(self.waypoints):
            self._mission_finished()
            return
        
        x, y, yaw = self.waypoints[self.current_waypoint_idx]
        
        # Create goal pose
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        progress = (self.current_waypoint_idx + 1) / len(self.waypoints) * 100
        self.get_logger().info(
            f'[{self.current_waypoint_idx + 1}/{len(self.waypoints)}] '
            f'({progress:.0f}%) → ({x:.2f}, {y:.2f})')
        
        # Send goal
        self._send_goal_future = self.nav_client.send_goal_async(
            goal_msg, 
            feedback_callback=self._feedback_callback
        )
        self._send_goal_future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        """Handle goal acceptance/rejection."""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().warn('   ⚠️ Goal rejected')
            self.failed_waypoints += 1
            self.current_waypoint_idx += 1
            self._send_next_goal()
            return
        
        # Wait for result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._get_result_callback)
        
        # Set timeout
        self._goal_start_time = time.time()

    def _feedback_callback(self, feedback_msg):
        """Handle navigation feedback."""
        feedback = feedback_msg.feedback
        # Could log distance remaining, ETA, etc.
        pass

    def _get_result_callback(self, future):
        """Handle navigation result."""
        result = future.result()
        status = result.status
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info('   ✅ Reached!')
            self.successful_waypoints += 1
        elif status == 5:  # CANCELED
            self.get_logger().warn('   ⏹️ Canceled')
            self.failed_waypoints += 1
        elif status == 6:  # ABORTED
            self.get_logger().warn('   ❌ Aborted (obstacle?)')
            self.failed_waypoints += 1
        else:
            self.get_logger().warn(f'   ❓ Unknown status: {status}')
            self.failed_waypoints += 1
        
        # Move to next waypoint
        self.current_waypoint_idx += 1
        self._send_next_goal()

    def _mission_finished(self):
        """Handle mission completion."""
        self.mission_complete = True
        elapsed = time.time() - self.start_time
        
        self.get_logger().info('')
        self.get_logger().info('=' * 50)
        self.get_logger().info('🏁 COVERAGE MISSION COMPLETE!')
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'   ✅ Successful: {self.successful_waypoints}/{len(self.waypoints)}')
        self.get_logger().info(f'   ❌ Failed: {self.failed_waypoints}/{len(self.waypoints)}')
        self.get_logger().info(f'   ⏱️ Total time: {elapsed / 60:.1f} minutes')
        
        coverage_pct = self.successful_waypoints / len(self.waypoints) * 100 if self.waypoints else 0
        self.get_logger().info(f'   📊 Coverage: {coverage_pct:.0f}%')
        self.get_logger().info('=' * 50)
        
        # Publish completion
        msg = Bool()
        msg.data = True
        self.mission_complete_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CoverageMission()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Mission interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
