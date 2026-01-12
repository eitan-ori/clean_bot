#!/usr/bin/env python3
"""
Frontier-Based Autonomous Explorer

This node explores an unknown environment by detecting and navigating to
"frontiers" - boundaries between known free space and unknown space.

The robot will autonomously:
1. Build a map using SLAM
2. Detect frontiers (unexplored areas)
3. Navigate to the nearest/best frontier
4. Repeat until no frontiers remain (map is complete)

This is a true autonomous exploration - no predefined waypoints needed!

Control (via /exploration_control topic, std_msgs/String):
- "start"  : Start/resume exploration
- "stop"   : Stop exploration
- "pause"  : Pause exploration (can resume)
- "resume" : Resume from pause
- "reset"  : Reset exploration state

Topics Subscribed:
- /map (nav_msgs/OccupancyGrid) - Map from SLAM
- /exploration_control (std_msgs/String) - External control commands

Topics Published:
- /exploration_complete (std_msgs/Bool) - True when done
- /frontiers (visualization_msgs/MarkerArray) - For RViz visualization

Author: Clean Bot Team
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray

from scipy import ndimage
from collections import deque


# Occupancy grid values
UNKNOWN = -1
FREE = 0
OCCUPIED = 100
FREE_THRESHOLD = 50


class ExplorationState:
    """Exploration state enumeration."""
    IDLE = 'IDLE'           # Waiting for start command
    EXPLORING = 'EXPLORING' # Actively exploring
    PAUSED = 'PAUSED'       # Temporarily paused
    STOPPED = 'STOPPED'     # Stopped by user command
    COMPLETE = 'COMPLETE'   # Exploration finished


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')

        # ===================== Parameters =====================
        self.declare_parameter('min_frontier_size', 5)       # Min cells for valid frontier
        self.declare_parameter('robot_radius', 0.18)         # For safety margin
        self.declare_parameter('exploration_timeout', 600.0) # 10 minutes max
        self.declare_parameter('goal_tolerance', 0.3)        # How close to get to frontier
        self.declare_parameter('min_goal_distance', 0.5)     # Don't go to very close frontiers
        self.declare_parameter('navigation_timeout', 60.0)   # Single goal timeout
        self.declare_parameter('auto_start', True)           # Start automatically when map received
        
        self.min_frontier_size = self.get_parameter('min_frontier_size').value
        self.robot_radius = self.get_parameter('robot_radius').value
        self.exploration_timeout = self.get_parameter('exploration_timeout').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.min_goal_distance = self.get_parameter('min_goal_distance').value
        self.navigation_timeout = self.get_parameter('navigation_timeout').value
        self.auto_start = self.get_parameter('auto_start').value

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
        
        # Control subscriber
        self.control_sub = self.create_subscription(
            String, 'exploration_control', self.control_callback, 10)

        # ===================== Publishers =====================
        self.exploration_complete_pub = self.create_publisher(
            Bool, 'exploration_complete', 10)
        self.frontier_markers_pub = self.create_publisher(
            MarkerArray, 'frontiers', 10)
        self.state_pub = self.create_publisher(
            String, 'exploration_state', 10)

        # ===================== State =====================
        self.exploration_state = ExplorationState.IDLE
        self.map_data = None
        self.map_info = None
        self.map_array = None
        
        self.robot_pose = None  # Will be updated from TF or odom
        self.current_goal = None
        self.is_navigating = False
        self.exploration_complete = False
        
        self.frontiers = []
        self.visited_frontiers = set()
        self.failed_goals = set()
        
        self.start_time = None
        self.navigation_start_time = None  # Track when current nav started
        self.current_goal_handle = None    # To cancel goals
        self.goals_attempted = 0
        self.goals_reached = 0
        self.consecutive_failures = 0      # Track consecutive navigation failures
        self.max_consecutive_failures = 3  # Finish if too many failures in a row

        # ===================== Timer =====================
        # Main exploration loop
        self.create_timer(2.0, self.exploration_loop)
        # State publisher
        self.create_timer(1.0, self.publish_state)

        self.get_logger().info('=' * 60)
        self.get_logger().info('🔍 Frontier Explorer Started')
        self.get_logger().info(f'   Waiting for start command or map (auto_start={self.auto_start})...')
        self.get_logger().info('   Robot will autonomously explore unknown areas')
        self.get_logger().info('=' * 60)

    def control_callback(self, msg: String):
        """Handle external control commands."""
        command = msg.data.lower().strip()
        self.get_logger().info(f'📬 Explorer received command: "{command}"')
        
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
        """Start exploration."""
        if self.exploration_state in [ExplorationState.IDLE, ExplorationState.STOPPED]:
            self.get_logger().info('▶️ Starting exploration...')
            self.exploration_state = ExplorationState.EXPLORING
            if self.start_time is None:
                self.start_time = self.get_clock().now()
        elif self.exploration_state == ExplorationState.PAUSED:
            self.handle_resume()
        else:
            self.get_logger().info(f'   Already in state: {self.exploration_state}')

    def handle_stop(self):
        """Stop exploration."""
        if self.exploration_state in [ExplorationState.EXPLORING, ExplorationState.PAUSED]:
            self.get_logger().info('🛑 Stopping exploration...')
            self.cancel_current_goal()
            self.exploration_state = ExplorationState.STOPPED
        else:
            self.get_logger().info(f'   Already in state: {self.exploration_state}')

    def handle_pause(self):
        """Pause exploration."""
        if self.exploration_state == ExplorationState.EXPLORING:
            self.get_logger().info('⏸️ Pausing exploration...')
            self.cancel_current_goal()
            self.exploration_state = ExplorationState.PAUSED
        else:
            self.get_logger().info(f'   Cannot pause from state: {self.exploration_state}')

    def handle_resume(self):
        """Resume exploration from pause."""
        if self.exploration_state == ExplorationState.PAUSED:
            self.get_logger().info('▶️ Resuming exploration...')
            self.exploration_state = ExplorationState.EXPLORING
        elif self.exploration_state == ExplorationState.STOPPED:
            self.handle_start()
        else:
            self.get_logger().info(f'   Cannot resume from state: {self.exploration_state}')

    def handle_reset(self):
        """Reset exploration state."""
        self.get_logger().info('🔄 Resetting exploration...')
        self.cancel_current_goal()
        
        # Reset all state
        self.exploration_state = ExplorationState.IDLE
        self.exploration_complete = False
        self.is_navigating = False
        self.frontiers = []
        self.visited_frontiers = set()
        self.failed_goals = set()
        self.start_time = None
        self.goals_attempted = 0
        self.goals_reached = 0
        self.consecutive_failures = 0
        self.current_goal = None
        self._retry_after_clear = False

    def publish_state(self):
        """Publish current exploration state."""
        msg = String()
        msg.data = self.exploration_state
        self.state_pub.publish(msg)

    def map_callback(self, msg: OccupancyGrid):
        """Process incoming map and convert to numpy array."""
        self.map_info = msg.info
        self.map_data = msg.data
        
        # Convert to numpy array for easier processing
        self.map_array = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width))
        
        if self.start_time is None and self.auto_start:
            self.start_time = self.get_clock().now()
            self.exploration_state = ExplorationState.EXPLORING
            self.get_logger().info(f'📍 First map received: {msg.info.width}x{msg.info.height}')
            self.get_logger().info('   Auto-starting exploration...')

    def exploration_loop(self):
        """Main exploration state machine."""
        # Only explore if in EXPLORING state
        if self.exploration_state != ExplorationState.EXPLORING:
            return
            
        if self.exploration_complete or self.map_array is None:
            return

        # Check timeout
        if self.start_time:
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            if elapsed > self.exploration_timeout:
                self.get_logger().warn('⏱️ Exploration timeout reached!')
                self.finish_exploration()
                return

        # If currently navigating, check for navigation timeout
        if self.is_navigating:
            if self.navigation_start_time:
                nav_elapsed = (self.get_clock().now() - self.navigation_start_time).nanoseconds / 1e9
                if nav_elapsed > self.navigation_timeout:
                    self.get_logger().warn(f'⏱️ Navigation timeout ({self.navigation_timeout}s) - canceling goal')
                    self.cancel_current_goal()
            return

        # Find frontiers
        self.frontiers = self.find_frontiers()
        self.publish_frontier_markers()

        if not self.frontiers:
            self.get_logger().info('✅ No more frontiers - exploration complete!')
            self.finish_exploration()
            return

        # Select best frontier
        best_frontier = self.select_best_frontier()
        
        if best_frontier is None:
            self.get_logger().warn('⚠️ Could not find reachable frontier')
            # Check if we've tried clearing failed goals already
            if not hasattr(self, '_retry_after_clear') or not self._retry_after_clear:
                self.get_logger().info('🔄 Clearing failed goals and retrying once...')
                self.failed_goals.clear()
                self._retry_after_clear = True
                return
            else:
                # We already tried clearing - all frontiers are unreachable
                self.get_logger().info('✅ All remaining frontiers are unreachable - exploration complete!')
                self.finish_exploration()
                return

        # Reset retry flag when we successfully find a frontier
        self._retry_after_clear = False
        
        # Navigate to frontier
        self.navigate_to_frontier(best_frontier)

    def find_frontiers(self) -> list:
        """
        Find frontier cells - boundaries between free and unknown space.
        Returns list of frontier centroids [(x, y), ...]
        """
        if self.map_array is None:
            return []

        height, width = self.map_array.shape
        
        # Create masks
        free_mask = self.map_array < FREE_THRESHOLD
        free_mask &= self.map_array >= 0  # Exclude unknown (-1)
        unknown_mask = self.map_array == UNKNOWN
        
        # Find frontier cells: free cells adjacent to unknown cells
        # Dilate unknown region and intersect with free space
        kernel = np.ones((3, 3), dtype=np.uint8)
        unknown_dilated = ndimage.binary_dilation(unknown_mask, kernel)
        
        frontier_mask = free_mask & unknown_dilated
        
        # Label connected frontier regions
        labeled, num_features = ndimage.label(frontier_mask)
        
        frontiers = []
        for i in range(1, num_features + 1):
            # Get cells in this frontier
            cells = np.where(labeled == i)
            size = len(cells[0])
            
            if size < self.min_frontier_size:
                continue
            
            # Calculate centroid in map coordinates
            centroid_row = np.mean(cells[0])
            centroid_col = np.mean(cells[1])
            
            # Convert to world coordinates
            x = self.map_info.origin.position.x + (centroid_col + 0.5) * self.map_info.resolution
            y = self.map_info.origin.position.y + (centroid_row + 0.5) * self.map_info.resolution
            
            frontiers.append({
                'x': x,
                'y': y,
                'size': size,
                'cells': list(zip(cells[0], cells[1]))
            })

        self.get_logger().debug(f'Found {len(frontiers)} frontiers')
        return frontiers

    def select_best_frontier(self):
        """
        Select the best frontier to explore.
        Strategy: Balance between distance and frontier size.
        """
        if not self.frontiers:
            return None

        # Get current robot position (approximate from map center if unknown)
        robot_x, robot_y = self.get_robot_position()
        
        best_frontier = None
        best_score = float('-inf')
        
        for frontier in self.frontiers:
            # Skip if we've failed to reach this area before
            key = (round(frontier['x'], 1), round(frontier['y'], 1))
            if key in self.failed_goals:
                continue
            
            # Calculate distance
            dx = frontier['x'] - robot_x
            dy = frontier['y'] - robot_y
            distance = math.sqrt(dx * dx + dy * dy)
            
            # Skip very close frontiers (likely unreachable due to obstacles)
            if distance < self.min_goal_distance:
                continue
            
            # Score: prefer larger frontiers that are closer
            # Normalize size and distance
            size_score = frontier['size'] / 100.0  # Normalize
            distance_score = 1.0 / (distance + 0.1)  # Inverse distance
            
            # Combined score (tune weights as needed)
            score = size_score * 0.3 + distance_score * 0.7
            
            if score > best_score:
                best_score = score
                best_frontier = frontier

        return best_frontier

    def get_robot_position(self) -> tuple:
        """Get current robot position. Returns (x, y)."""
        # TODO: Get from TF or /odom topic
        # For now, estimate from map center or last goal
        if self.current_goal:
            return (self.current_goal['x'], self.current_goal['y'])
        
        # Default: map center
        if self.map_info:
            cx = self.map_info.origin.position.x + \
                 (self.map_info.width * self.map_info.resolution) / 2
            cy = self.map_info.origin.position.y + \
                 (self.map_info.height * self.map_info.resolution) / 2
            return (cx, cy)
        
        return (0.0, 0.0)

    def navigate_to_frontier(self, frontier: dict):
        """Send navigation goal to frontier."""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('❌ Nav2 action server not available!')
            return

        # Find a safe goal point near the frontier (not exactly on it)
        goal_x, goal_y = self.find_safe_goal_near_frontier(frontier)
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = goal_x
        goal_msg.pose.pose.position.y = goal_y
        goal_msg.pose.pose.orientation.w = 1.0

        self.current_goal = {'x': goal_x, 'y': goal_y}
        self.is_navigating = True
        self.navigation_start_time = self.get_clock().now()  # Track nav start
        self.goals_attempted += 1

        self.get_logger().info(
            f'🚀 Exploring frontier #{self.goals_attempted} at ({goal_x:.2f}, {goal_y:.2f}) '
            f'[size: {frontier["size"]} cells]')

        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def cancel_current_goal(self):
        """Cancel the current navigation goal."""
        if self.current_goal_handle is not None:
            self.get_logger().info('   🛑 Canceling current goal...')
            cancel_future = self.current_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
        else:
            # No handle, just reset state
            self.is_navigating = False
            self.navigation_start_time = None
            if self.current_goal:
                key = (round(self.current_goal['x'], 1), round(self.current_goal['y'], 1))
                self.failed_goals.add(key)

    def cancel_done_callback(self, future):
        """Called when goal cancellation completes."""
        self.get_logger().info('   🛑 Goal canceled')
        self.is_navigating = False
        self.navigation_start_time = None
        self.current_goal_handle = None
        if self.current_goal:
            key = (round(self.current_goal['x'], 1), round(self.current_goal['y'], 1))
            self.failed_goals.add(key)

    def find_safe_goal_near_frontier(self, frontier: dict) -> tuple:
        """
        Find a safe (free) cell near the frontier centroid.
        The goal should be in free space, not on the frontier itself.
        """
        target_x = frontier['x']
        target_y = frontier['y']
        
        # Convert to grid coordinates
        col = int((target_x - self.map_info.origin.position.x) / self.map_info.resolution)
        row = int((target_y - self.map_info.origin.position.y) / self.map_info.resolution)
        
        # Search in expanding circles for a free cell
        robot_x, robot_y = self.get_robot_position()
        
        for radius in range(1, 20):  # Search up to 20 cells away
            for dr in range(-radius, radius + 1):
                for dc in range(-radius, radius + 1):
                    r, c = row + dr, col + dc
                    
                    if 0 <= r < self.map_info.height and 0 <= c < self.map_info.width:
                        if 0 <= self.map_array[r, c] < FREE_THRESHOLD:
                            # Found free cell - convert back to world coords
                            x = self.map_info.origin.position.x + (c + 0.5) * self.map_info.resolution
                            y = self.map_info.origin.position.y + (r + 0.5) * self.map_info.resolution
                            return (x, y)
        
        # Fallback: return original point
        return (target_x, target_y)

    def goal_response_callback(self, future):
        """Handle goal acceptance/rejection."""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().warn('   ⚠️ Goal rejected by Nav2')
            self.is_navigating = False
            self.navigation_start_time = None
            if self.current_goal:
                key = (round(self.current_goal['x'], 1), round(self.current_goal['y'], 1))
                self.failed_goals.add(key)
            return

        self.current_goal_handle = goal_handle  # Save for potential cancellation
        self.get_logger().info('   📍 Goal accepted, navigating...')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """Handle navigation result."""
        result = future.result()
        status = result.status
        
        self.is_navigating = False
        self.navigation_start_time = None
        self.current_goal_handle = None
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info('   ✅ Reached exploration point!')
            self.goals_reached += 1
            self.consecutive_failures = 0  # Reset failure counter on success
        elif status == 6:  # ABORTED
            self.get_logger().warn('   ❌ Navigation aborted (obstacle?)')
            self.consecutive_failures += 1
            if self.current_goal:
                key = (round(self.current_goal['x'], 1), round(self.current_goal['y'], 1))
                self.failed_goals.add(key)
            # Check if too many consecutive failures
            if self.consecutive_failures >= self.max_consecutive_failures:
                self.get_logger().warn(f'⚠️ {self.consecutive_failures} consecutive failures - finishing exploration')
                self.finish_exploration()
        elif status == 5:  # CANCELED
            self.get_logger().info('   🛑 Navigation was canceled')
            self.consecutive_failures += 1
        else:
            self.get_logger().warn(f'   ❓ Navigation ended with status: {status}')
            self.consecutive_failures += 1

    def publish_frontier_markers(self):
        """Publish frontier markers for RViz visualization."""
        marker_array = MarkerArray()
        
        # Clear old markers
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        
        for i, frontier in enumerate(self.frontiers):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'frontiers'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = frontier['x']
            marker.pose.position.y = frontier['y']
            marker.pose.position.z = 0.1
            marker.pose.orientation.w = 1.0
            
            # Size based on frontier size
            size = min(0.5, max(0.1, frontier['size'] / 50.0))
            marker.scale.x = size
            marker.scale.y = size
            marker.scale.z = size
            
            # Color: blue
            marker.color.r = 0.0
            marker.color.g = 0.5
            marker.color.b = 1.0
            marker.color.a = 0.8
            
            marker_array.markers.append(marker)
        
        self.frontier_markers_pub.publish(marker_array)

    def finish_exploration(self):
        """Called when exploration is complete."""
        self.exploration_complete = True
        self.exploration_state = ExplorationState.COMPLETE
        
        elapsed = 0
        if self.start_time:
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info('🏁 EXPLORATION COMPLETE!')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'   ⏱️ Time: {elapsed / 60:.1f} minutes')
        self.get_logger().info(f'   🎯 Goals attempted: {self.goals_attempted}')
        self.get_logger().info(f'   ✅ Goals reached: {self.goals_reached}')
        self.get_logger().info('')
        self.get_logger().info('💡 Map is ready! You can now:')
        self.get_logger().info('   1. Save it: ros2 run nav2_map_server map_saver_cli -f ~/my_map')
        self.get_logger().info('   2. Start coverage: ros2 topic pub --once /mission_command std_msgs/msg/String "data: \'start_clean\'"')
        self.get_logger().info('=' * 60)
        
        # Publish completion
        msg = Bool()
        msg.data = True
        self.exploration_complete_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Exploration interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
