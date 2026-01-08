#!/usr/bin/env python3
"""
Adaptive Coverage Planner for Complex Room Shapes

This node generates a coverage path for cleaning that adapts to:
- Irregular room shapes
- Furniture and obstacles
- Multiple rooms connected together

Algorithm: Cell Decomposition + Boustrophedon
1. Decompose free space into cells
2. Generate zigzag path within each cell
3. Connect cells optimally (TSP-like)

The path follows actual free space from the map, not a simple rectangle!

Topics Subscribed:
- /map (nav_msgs/OccupancyGrid) - Complete map from SLAM
- /exploration_complete (std_msgs/Bool) - Trigger to start coverage

Actions Used:
- /navigate_to_pose (nav2_msgs/NavigateToPose)

Parameters:
- coverage_width: Width of cleaning stripe (default: 0.14m = 14cm)
- start_on_exploration_complete: Auto-start when exploration finishes

Author: Clean Bot Team
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker, MarkerArray

from scipy import ndimage
from collections import deque
import heapq


# Occupancy grid values
FREE_THRESHOLD = 50
OCCUPIED_THRESHOLD = 65


class AdaptiveCoveragePlanner(Node):
    def __init__(self):
        super().__init__('adaptive_coverage_planner')

        # ===================== Parameters =====================
        self.declare_parameter('coverage_width', 0.14)           # 14cm cleaning width
        self.declare_parameter('overlap_ratio', 0.15)            # 15% overlap for safety
        self.declare_parameter('robot_radius', 0.12)             # Robot radius for inflation
        self.declare_parameter('timeout_per_waypoint', 90)       # Seconds per goal
        self.declare_parameter('start_on_exploration_complete', True)
        self.declare_parameter('min_region_area', 0.1)           # Min area to cover (m²)
        
        self.coverage_width = self.get_parameter('coverage_width').value
        self.overlap_ratio = self.get_parameter('overlap_ratio').value
        self.robot_radius = self.get_parameter('robot_radius').value
        self.timeout = self.get_parameter('timeout_per_waypoint').value
        self.auto_start = self.get_parameter('start_on_exploration_complete').value
        self.min_region_area = self.get_parameter('min_region_area').value
        
        # Effective step between lines
        self.step_size = self.coverage_width * (1 - self.overlap_ratio)

        # ===================== Action Client =====================
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

        # ===================== Publishers =====================
        self.coverage_complete_pub = self.create_publisher(Bool, 'coverage_complete', 10)
        self.coverage_path_pub = self.create_publisher(Path, 'coverage_path', 10)
        self.waypoint_markers_pub = self.create_publisher(MarkerArray, 'coverage_waypoints', 10)

        # ===================== State =====================
        self.map_array = None
        self.map_info = None
        self.inflated_map = None
        
        self.waypoints = []
        self.current_waypoint_idx = 0
        self.is_navigating = False
        self.mission_started = False
        self.mission_complete = False
        
        # Statistics
        self.successful_waypoints = 0
        self.failed_waypoints = 0
        self.start_time = None
        self.total_distance = 0.0

        self.get_logger().info('=' * 60)
        self.get_logger().info('🧹 Adaptive Coverage Planner Started')
        self.get_logger().info(f'   Coverage width: {self.coverage_width * 100:.0f}cm')
        self.get_logger().info(f'   Step size: {self.step_size * 100:.1f}cm')
        self.get_logger().info('   Waiting for map...')
        self.get_logger().info('=' * 60)

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
        if self.map_array is None:
            self.get_logger().error('❌ No map available!')
            return
        
        if self.mission_started:
            self.get_logger().warn('⚠️ Mission already in progress')
            return

        self.get_logger().info('')
        self.get_logger().info('🔧 Generating coverage path...')
        
        # Step 1: Inflate obstacles for robot safety
        self.inflate_obstacles()
        
        # Step 2: Generate adaptive coverage path
        self.waypoints = self.generate_adaptive_coverage_path()
        
        if not self.waypoints:
            self.get_logger().error('❌ Could not generate coverage path!')
            return
        
        # Step 3: Publish path for visualization
        self.publish_coverage_path()
        self.publish_waypoint_markers()
        
        # Step 4: Start mission
        self.mission_started = True
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
        Generate coverage path that follows actual room shape.
        Uses cell decomposition: divides space into vertical strips
        and generates zigzag within each strip.
        """
        waypoints = []
        
        height, width = self.inflated_map.shape
        resolution = self.map_info.resolution
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y
        
        # Calculate step in cells
        step_cells = max(1, int(self.step_size / resolution))
        
        # Find all free columns and their vertical extents
        going_up = True
        
        for col in range(0, width, step_cells):
            # Find free segments in this column
            segments = self.find_free_segments_in_column(col)
            
            if not segments:
                continue
            
            # Convert to world coordinates and add waypoints
            x = origin_x + (col + 0.5) * resolution
            
            for seg_start, seg_end in segments:
                y_start = origin_y + (seg_start + 0.5) * resolution
                y_end = origin_y + (seg_end + 0.5) * resolution
                
                if going_up:
                    waypoints.append((x, y_start, math.pi / 2))  # Facing up
                    waypoints.append((x, y_end, math.pi / 2))
                else:
                    waypoints.append((x, y_end, -math.pi / 2))  # Facing down
                    waypoints.append((x, y_start, -math.pi / 2))
            
            if segments:  # Only flip if we added waypoints
                going_up = not going_up
        
        # Optimize path order (simple greedy nearest neighbor)
        if len(waypoints) > 2:
            waypoints = self.optimize_path_order(waypoints)
        
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
                
                # Only add if segment is long enough
                segment_length = (seg_end - seg_start) * self.map_info.resolution
                if segment_length >= self.step_size * 2:  # At least 2 steps
                    segments.append((seg_start, seg_end))
        
        # Handle segment that goes to end of column
        if in_segment:
            seg_end = height - 1
            segment_length = (seg_end - seg_start) * self.map_info.resolution
            if segment_length >= self.step_size * 2:
                segments.append((seg_start, seg_end))
        
        return segments

    def optimize_path_order(self, waypoints: list) -> list:
        """
        Reorder waypoints to minimize travel distance.
        Simple greedy nearest-neighbor approach.
        """
        if len(waypoints) <= 2:
            return waypoints
        
        # Start from first waypoint
        ordered = [waypoints[0]]
        remaining = list(waypoints[1:])
        
        while remaining:
            last = ordered[-1]
            
            # Find nearest remaining waypoint
            min_dist = float('inf')
            nearest_idx = 0
            
            for i, wp in enumerate(remaining):
                dist = math.sqrt((wp[0] - last[0])**2 + (wp[1] - last[1])**2)
                if dist < min_dist:
                    min_dist = dist
                    nearest_idx = i
            
            ordered.append(remaining.pop(nearest_idx))
        
        return ordered

    def send_next_goal(self):
        """Send next waypoint to Nav2."""
        if self.current_waypoint_idx >= len(self.waypoints):
            self.finish_mission()
            return

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('❌ Nav2 action server not available!')
            return

        x, y, yaw = self.waypoints[self.current_waypoint_idx]
        
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

        self.is_navigating = True
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection."""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().warn('   ⚠️ Goal rejected - skipping')
            self.failed_waypoints += 1
            self.is_navigating = False
            self.current_waypoint_idx += 1
            self.send_next_goal()
            return

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle navigation result."""
        result = future.result()
        status = result.status
        
        self.is_navigating = False
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info('   ✅')
            self.successful_waypoints += 1
        elif status == 6:  # ABORTED
            self.get_logger().warn('   ❌ (obstacle)')
            self.failed_waypoints += 1
        else:
            self.get_logger().warn(f'   ❓ status={status}')
            self.failed_waypoints += 1
        
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
        self.mission_complete = True
        
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
