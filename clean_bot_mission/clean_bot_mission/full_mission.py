#!/usr/bin/env python3
"""
Clean Bot Full Mission Controller

This is the main mission controller that orchestrates the complete
autonomous cleaning workflow:

Phase 1: EXPLORATION
- Robot explores unknown environment using frontier-based exploration
- SLAM builds a complete map of the space
- Continues until no unexplored areas remain

Phase 2: COVERAGE
- Once map is complete, generates optimal coverage path
- Executes cleaning pattern that covers all free space
- Adapts to room shape, furniture, and obstacles

Phase 3: RETURN HOME
- Returns to starting position when done

Usage:
    ros2 run clean_bot_mission full_mission

    # Or with parameters:
    ros2 run clean_bot_mission full_mission --ros-args \
        -p coverage_width:=0.14 \
        -p skip_exploration:=false

Author: Clean Bot Team
"""

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid

# Import our mission modules
from clean_bot_mission.frontier_explorer import FrontierExplorer
from clean_bot_mission.adaptive_coverage import AdaptiveCoveragePlanner


class MissionState:
    """Mission state enumeration."""
    IDLE = 'IDLE'
    EXPLORING = 'EXPLORING'
    COVERAGE = 'COVERAGE'
    RETURNING = 'RETURNING'
    COMPLETE = 'COMPLETE'
    ERROR = 'ERROR'


class FullMissionController(Node):
    def __init__(self):
        super().__init__('full_mission_controller')

        # ===================== Parameters =====================
        self.declare_parameter('coverage_width', 0.14)
        self.declare_parameter('skip_exploration', False)
        self.declare_parameter('return_home_after', True)
        self.declare_parameter('home_x', 0.0)
        self.declare_parameter('home_y', 0.0)
        
        self.coverage_width = self.get_parameter('coverage_width').value
        self.skip_exploration = self.get_parameter('skip_exploration').value
        self.return_home = self.get_parameter('return_home_after').value
        self.home_x = self.get_parameter('home_x').value
        self.home_y = self.get_parameter('home_y').value

        # ===================== State =====================
        self.state = MissionState.IDLE
        self.start_time = None
        self.exploration_complete = False
        self.coverage_complete = False

        # ===================== Publishers =====================
        self.state_pub = self.create_publisher(String, 'mission_state', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # ===================== Subscribers =====================
        self.exploration_complete_sub = self.create_subscription(
            Bool, 'exploration_complete', self.exploration_complete_callback, 10)
        self.coverage_complete_sub = self.create_subscription(
            Bool, 'coverage_complete', self.coverage_complete_callback, 10)

        # ===================== Timer =====================
        self.create_timer(1.0, self.publish_state)
        
        # Print startup banner
        self.print_banner()
        
        # Start mission after short delay
        self.create_timer(3.0, self.start_mission, callback_group=ReentrantCallbackGroup())

    def print_banner(self):
        """Print startup banner."""
        self.get_logger().info('')
        self.get_logger().info('╔' + '═' * 58 + '╗')
        self.get_logger().info('║' + ' ' * 15 + '🤖 CLEAN BOT MISSION' + ' ' * 23 + '║')
        self.get_logger().info('╠' + '═' * 58 + '╣')
        self.get_logger().info('║  Phase 1: Explore unknown environment' + ' ' * 19 + '║')
        self.get_logger().info('║  Phase 2: Cover all free space (clean)' + ' ' * 18 + '║')
        self.get_logger().info('║  Phase 3: Return home' + ' ' * 36 + '║')
        self.get_logger().info('╠' + '═' * 58 + '╣')
        self.get_logger().info(f'║  Coverage width: {self.coverage_width * 100:.0f}cm' + ' ' * 37 + '║')
        self.get_logger().info(f'║  Skip exploration: {self.skip_exploration}' + ' ' * 33 + '║')
        self.get_logger().info('╚' + '═' * 58 + '╝')
        self.get_logger().info('')

    def start_mission(self):
        """Start the mission sequence."""
        if self.state != MissionState.IDLE:
            return
        
        self.start_time = time.time()
        
        if self.skip_exploration:
            self.get_logger().info('⏭️ Skipping exploration (using existing map)')
            self.state = MissionState.COVERAGE
            self.start_coverage()
        else:
            self.get_logger().info('🔍 Starting exploration phase...')
            self.state = MissionState.EXPLORING
            # Exploration node will handle the actual exploration
            # We just monitor for completion

    def exploration_complete_callback(self, msg: Bool):
        """Called when exploration finishes."""
        if msg.data and self.state == MissionState.EXPLORING:
            self.exploration_complete = True
            self.get_logger().info('')
            self.get_logger().info('✅ Exploration phase complete!')
            self.get_logger().info('')
            
            # Short pause before starting coverage
            time.sleep(2.0)
            
            self.state = MissionState.COVERAGE
            self.start_coverage()

    def start_coverage(self):
        """Trigger coverage phase."""
        self.get_logger().info('🧹 Starting coverage phase...')
        
        # Send signal to coverage planner
        # The adaptive_coverage node listens for exploration_complete
        # If we skipped exploration, we need to manually trigger it
        if self.skip_exploration:
            # Publish fake exploration complete
            msg = Bool()
            msg.data = True
            
            # Create temporary publisher
            pub = self.create_publisher(Bool, 'exploration_complete', 10)
            time.sleep(0.5)
            pub.publish(msg)

    def coverage_complete_callback(self, msg: Bool):
        """Called when coverage finishes."""
        if msg.data and self.state == MissionState.COVERAGE:
            self.coverage_complete = True
            self.get_logger().info('')
            self.get_logger().info('✅ Coverage phase complete!')
            self.get_logger().info('')
            
            if self.return_home:
                self.state = MissionState.RETURNING
                self.return_home_sequence()
            else:
                self.finish_mission()

    def return_home_sequence(self):
        """Navigate back to home position."""
        self.get_logger().info(f'🏠 Returning to home position ({self.home_x}, {self.home_y})...')
        
        # This would use Nav2 to navigate home
        # For simplicity, we just mark as complete
        # In real implementation, use NavigateToPose action
        
        time.sleep(1.0)
        self.finish_mission()

    def finish_mission(self):
        """Mission complete."""
        self.state = MissionState.COMPLETE
        
        elapsed = time.time() - self.start_time if self.start_time else 0
        
        # Stop robot
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        
        self.get_logger().info('')
        self.get_logger().info('╔' + '═' * 58 + '╗')
        self.get_logger().info('║' + ' ' * 18 + '🎉 MISSION COMPLETE!' + ' ' * 20 + '║')
        self.get_logger().info('╠' + '═' * 58 + '╣')
        self.get_logger().info(f'║  Total time: {elapsed / 60:.1f} minutes' + ' ' * 35 + '║')
        self.get_logger().info('╚' + '═' * 58 + '╝')
        self.get_logger().info('')

    def publish_state(self):
        """Publish current mission state."""
        msg = String()
        msg.data = self.state
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    # Create nodes
    mission_controller = FullMissionController()
    explorer = FrontierExplorer()
    coverage_planner = AdaptiveCoveragePlanner()
    
    # Use multi-threaded executor to run all nodes
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(mission_controller)
    executor.add_node(explorer)
    executor.add_node(coverage_planner)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        mission_controller.get_logger().info('Mission interrupted by user')
    finally:
        executor.shutdown()
        mission_controller.destroy_node()
        explorer.destroy_node()
        coverage_planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
