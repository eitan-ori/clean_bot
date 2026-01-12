#!/usr/bin/env python3
"""
Clean Bot Full Mission Controller

This is the main mission controller that orchestrates the complete
autonomous cleaning workflow:

Phase 1: WAITING_FOR_SCAN
- Robot waits for external command to start exploration

Phase 2: EXPLORATION
- Robot explores unknown environment using frontier-based exploration
- SLAM builds a complete map of the space
- Continues until no unexplored areas remain OR external stop command

Phase 3: WAITING_FOR_CLEAN
- Robot waits for external command to start cleaning

Phase 4: COVERAGE
- Once map is complete, generates optimal coverage path
- Executes cleaning pattern that covers all free space
- Adapts to room shape, furniture, and obstacles
- Can be stopped via external command

Phase 5: RETURN HOME
- Returns to starting position when done

Commands (via /mission_command topic, std_msgs/String):
- "start_scan"   : Start exploration phase
- "stop_scan"    : Stop exploration and wait for clean command
- "start_clean"  : Start coverage/cleaning phase
- "stop_clean"   : Stop cleaning
- "go_home"      : Return to home position
- "reset"        : Reset to initial WAITING_FOR_SCAN state

Usage:
    ros2 run clean_bot_mission full_mission

    # Control via topic:
    ros2 topic pub --once /mission_command std_msgs/msg/String "data: 'start_scan'"
    ros2 topic pub --once /mission_command std_msgs/msg/String "data: 'stop_scan'"
    ros2 topic pub --once /mission_command std_msgs/msg/String "data: 'start_clean'"
    ros2 topic pub --once /mission_command std_msgs/msg/String "data: 'stop_clean'"

    # Or with parameters:
    ros2 run clean_bot_mission full_mission --ros-args \
        -p coverage_width:=0.14 \
        -p auto_start:=false

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
    WAITING_FOR_SCAN = 'WAITING_FOR_SCAN'     # Waiting for start_scan command
    EXPLORING = 'EXPLORING'                     # Currently exploring/scanning
    WAITING_FOR_CLEAN = 'WAITING_FOR_CLEAN'   # Exploration done, waiting for start_clean
    COVERAGE = 'COVERAGE'                       # Currently cleaning
    RETURNING = 'RETURNING'                     # Going back home
    COMPLETE = 'COMPLETE'                       # Mission finished
    PAUSED = 'PAUSED'                          # Temporarily paused
    ERROR = 'ERROR'                            # Error state


class FullMissionController(Node):
    def __init__(self):
        super().__init__('full_mission_controller')

        # ===================== Parameters =====================
        self.declare_parameter('coverage_width', 0.14)
        self.declare_parameter('auto_start', False)           # If True, start scan immediately
        self.declare_parameter('return_home_after', True)
        self.declare_parameter('home_x', 0.0)
        self.declare_parameter('home_y', 0.0)
        
        self.coverage_width = self.get_parameter('coverage_width').value
        self.auto_start = self.get_parameter('auto_start').value
        self.return_home = self.get_parameter('return_home_after').value
        self.home_x = self.get_parameter('home_x').value
        self.home_y = self.get_parameter('home_y').value

        # ===================== State =====================
        self.state = MissionState.WAITING_FOR_SCAN
        self.previous_state = None  # For pause/resume
        self.start_time = None
        self.exploration_complete = False
        self.coverage_complete = False

        # ===================== Publishers =====================
        self.state_pub = self.create_publisher(String, 'mission_state', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Control publishers for sub-nodes
        self.exploration_control_pub = self.create_publisher(String, 'exploration_control', 10)
        self.coverage_control_pub = self.create_publisher(String, 'coverage_control', 10)
        self.exploration_complete_pub = self.create_publisher(Bool, 'exploration_complete', 10)

        # ===================== Subscribers =====================
        self.exploration_complete_sub = self.create_subscription(
            Bool, 'exploration_complete', self.exploration_complete_callback, 10)
        self.coverage_complete_sub = self.create_subscription(
            Bool, 'coverage_complete', self.coverage_complete_callback, 10)
        
        # Command subscriber for external control
        self.command_sub = self.create_subscription(
            String, 'mission_command', self.command_callback, 10)

        # ===================== Timer =====================
        self.create_timer(1.0, self.publish_state)
        
        # Print startup banner
        self.print_banner()
        
        # Auto-start if configured
        if self.auto_start:
            self.create_timer(3.0, self._auto_start_once, callback_group=ReentrantCallbackGroup())

    def _auto_start_once(self):
        """Auto-start exploration once."""
        if self.state == MissionState.WAITING_FOR_SCAN:
            self.get_logger().info('🚀 Auto-starting exploration...')
            self.start_exploration()
        # This timer fires once, we don't need to track it

    def print_banner(self):
        """Print startup banner."""
        self.get_logger().info('')
        self.get_logger().info('╔' + '═' * 58 + '╗')
        self.get_logger().info('║' + ' ' * 15 + '🤖 CLEAN BOT MISSION' + ' ' * 23 + '║')
        self.get_logger().info('╠' + '═' * 58 + '╣')
        self.get_logger().info('║  Phase 1: Wait for scan command' + ' ' * 25 + '║')
        self.get_logger().info('║  Phase 2: Explore unknown environment' + ' ' * 19 + '║')
        self.get_logger().info('║  Phase 3: Wait for clean command' + ' ' * 24 + '║')
        self.get_logger().info('║  Phase 4: Cover all free space (clean)' + ' ' * 18 + '║')
        self.get_logger().info('║  Phase 5: Return home' + ' ' * 36 + '║')
        self.get_logger().info('╠' + '═' * 58 + '╣')
        self.get_logger().info(f'║  Coverage width: {self.coverage_width * 100:.0f}cm' + ' ' * 37 + '║')
        self.get_logger().info(f'║  Auto-start: {self.auto_start}' + ' ' * 39 + '║')
        self.get_logger().info('╠' + '═' * 58 + '╣')
        self.get_logger().info('║  Commands (publish to /mission_command):' + ' ' * 16 + '║')
        self.get_logger().info('║    start_scan  - Begin exploration' + ' ' * 22 + '║')
        self.get_logger().info('║    stop_scan   - Stop exploration' + ' ' * 23 + '║')
        self.get_logger().info('║    start_clean - Begin cleaning' + ' ' * 25 + '║')
        self.get_logger().info('║    stop_clean  - Stop cleaning' + ' ' * 26 + '║')
        self.get_logger().info('║    go_home     - Return to home' + ' ' * 25 + '║')
        self.get_logger().info('║    reset       - Reset to initial state' + ' ' * 17 + '║')
        self.get_logger().info('╚' + '═' * 58 + '╝')
        self.get_logger().info('')
        self.get_logger().info('⏳ Waiting for "start_scan" command...')
        self.get_logger().info('   Send: ros2 topic pub --once /mission_command std_msgs/msg/String "data: \'start_scan\'"')
        self.get_logger().info('')

    def command_callback(self, msg: String):
        """Handle external mission commands."""
        command = msg.data.lower().strip()
        self.get_logger().info(f'📬 Received command: "{command}"')
        
        if command == 'start_scan':
            self.handle_start_scan()
        elif command == 'stop_scan':
            self.handle_stop_scan()
        elif command == 'start_clean':
            self.handle_start_clean()
        elif command == 'stop_clean':
            self.handle_stop_clean()
        elif command == 'go_home':
            self.handle_go_home()
        elif command == 'reset':
            self.handle_reset()
        elif command == 'pause':
            self.handle_pause()
        elif command == 'resume':
            self.handle_resume()
        else:
            self.get_logger().warn(f'❓ Unknown command: "{command}"')
            self.get_logger().info('   Valid commands: start_scan, stop_scan, start_clean, stop_clean, go_home, reset, pause, resume')

    def handle_start_scan(self):
        """Handle start_scan command."""
        if self.state == MissionState.WAITING_FOR_SCAN:
            self.start_exploration()
        elif self.state == MissionState.WAITING_FOR_CLEAN:
            # Go back to scanning from waiting for clean
            self.get_logger().info('🔄 Returning to exploration from waiting state...')
            self.exploration_complete = False
            self.start_exploration()
        else:
            self.get_logger().warn(f'⚠️ Cannot start scan in state: {self.state}')

    def handle_stop_scan(self):
        """Handle stop_scan command."""
        if self.state == MissionState.EXPLORING:
            self.get_logger().info('🛑 Stopping exploration...')
            self.stop_robot()
            
            # Tell explorer to stop
            ctrl_msg = String()
            ctrl_msg.data = 'stop'
            self.exploration_control_pub.publish(ctrl_msg)
            
            self.state = MissionState.WAITING_FOR_CLEAN
            self.get_logger().info('')
            self.get_logger().info('⏳ Exploration stopped. Waiting for "start_clean" command...')
            self.get_logger().info('   Send: ros2 topic pub --once /mission_command std_msgs/msg/String "data: \'start_clean\'"')
            self.get_logger().info('   Or go back to scanning: ros2 topic pub --once /mission_command std_msgs/msg/String "data: \'start_scan\'"')
            self.get_logger().info('')
        else:
            self.get_logger().warn(f'⚠️ Not currently scanning (state: {self.state})')

    def handle_start_clean(self):
        """Handle start_clean command."""
        if self.state == MissionState.WAITING_FOR_CLEAN:
            self.start_coverage()
        elif self.state == MissionState.WAITING_FOR_SCAN:
            self.get_logger().warn('⚠️ Must scan first before cleaning!')
        else:
            self.get_logger().warn(f'⚠️ Cannot start clean in state: {self.state}')

    def handle_stop_clean(self):
        """Handle stop_clean command."""
        if self.state == MissionState.COVERAGE:
            self.get_logger().info('🛑 Stopping cleaning...')
            self.stop_robot()
            
            # Tell coverage planner to stop
            ctrl_msg = String()
            ctrl_msg.data = 'stop'
            self.coverage_control_pub.publish(ctrl_msg)
            
            self.state = MissionState.WAITING_FOR_CLEAN
            self.get_logger().info('')
            self.get_logger().info('⏳ Cleaning stopped. Waiting for command...')
            self.get_logger().info('   Continue cleaning: ros2 topic pub --once /mission_command std_msgs/msg/String "data: \'start_clean\'"')
            self.get_logger().info('   Go back to scanning: ros2 topic pub --once /mission_command std_msgs/msg/String "data: \'start_scan\'"')
            self.get_logger().info('')
        else:
            self.get_logger().warn(f'⚠️ Not currently cleaning (state: {self.state})')

    def handle_go_home(self):
        """Handle go_home command."""
        self.get_logger().info('🏠 Going home...')
        self.stop_robot()
        
        # Stop any running operations
        ctrl_msg = String()
        ctrl_msg.data = 'stop'
        self.exploration_control_pub.publish(ctrl_msg)
        self.coverage_control_pub.publish(ctrl_msg)
        
        self.state = MissionState.RETURNING
        self.return_home_sequence()

    def handle_reset(self):
        """Handle reset command - go back to initial waiting state."""
        self.get_logger().info('🔄 Resetting mission...')
        self.stop_robot()
        
        # Stop any running operations
        ctrl_msg = String()
        ctrl_msg.data = 'stop'
        self.exploration_control_pub.publish(ctrl_msg)
        self.coverage_control_pub.publish(ctrl_msg)
        
        # Reset state
        self.state = MissionState.WAITING_FOR_SCAN
        self.exploration_complete = False
        self.coverage_complete = False
        self.start_time = None
        
        # Tell sub-nodes to reset
        ctrl_msg.data = 'reset'
        self.exploration_control_pub.publish(ctrl_msg)
        self.coverage_control_pub.publish(ctrl_msg)
        
        self.get_logger().info('')
        self.get_logger().info('✅ Mission reset. Waiting for "start_scan" command...')
        self.get_logger().info('')

    def handle_pause(self):
        """Handle pause command."""
        if self.state in [MissionState.EXPLORING, MissionState.COVERAGE]:
            self.get_logger().info('⏸️ Pausing mission...')
            self.previous_state = self.state
            self.stop_robot()
            
            # Tell sub-nodes to pause
            ctrl_msg = String()
            ctrl_msg.data = 'pause'
            self.exploration_control_pub.publish(ctrl_msg)
            self.coverage_control_pub.publish(ctrl_msg)
            
            self.state = MissionState.PAUSED
        else:
            self.get_logger().warn(f'⚠️ Cannot pause in state: {self.state}')

    def handle_resume(self):
        """Handle resume command."""
        if self.state == MissionState.PAUSED and self.previous_state:
            self.get_logger().info('▶️ Resuming mission...')
            
            # Tell sub-nodes to resume
            ctrl_msg = String()
            ctrl_msg.data = 'resume'
            self.exploration_control_pub.publish(ctrl_msg)
            self.coverage_control_pub.publish(ctrl_msg)
            
            self.state = self.previous_state
            self.previous_state = None
        else:
            self.get_logger().warn(f'⚠️ Cannot resume in state: {self.state}')

    def stop_robot(self):
        """Stop robot movement."""
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)

    def start_exploration(self):
        """Start the exploration phase."""
        self.start_time = time.time()
        self.state = MissionState.EXPLORING
        
        self.get_logger().info('')
        self.get_logger().info('🔍 Starting exploration phase...')
        self.get_logger().info('   Robot will autonomously explore the environment')
        self.get_logger().info('   Send "stop_scan" to stop and proceed to cleaning')
        self.get_logger().info('')
        
        # Tell explorer to start
        ctrl_msg = String()
        ctrl_msg.data = 'start'
        self.exploration_control_pub.publish(ctrl_msg)

    def exploration_complete_callback(self, msg: Bool):
        """Called when exploration finishes automatically."""
        if msg.data and self.state == MissionState.EXPLORING:
            self.exploration_complete = True
            self.get_logger().info('')
            self.get_logger().info('✅ Exploration phase complete!')
            self.get_logger().info('')
            
            # Move to waiting state instead of automatically starting coverage
            self.state = MissionState.WAITING_FOR_CLEAN
            self.get_logger().info('⏳ Waiting for "start_clean" command...')
            self.get_logger().info('   Send: ros2 topic pub --once /mission_command std_msgs/msg/String "data: \'start_clean\'"')
            self.get_logger().info('   Or continue scanning: ros2 topic pub --once /mission_command std_msgs/msg/String "data: \'start_scan\'"')
            self.get_logger().info('')

    def start_coverage(self):
        """Trigger coverage phase."""
        self.get_logger().info('')
        self.get_logger().info('🧹 Starting coverage/cleaning phase...')
        self.get_logger().info('   Robot will clean all free space')
        self.get_logger().info('   Send "stop_clean" to stop')
        self.get_logger().info('')
        
        self.state = MissionState.COVERAGE
        
        # Publish exploration complete to trigger coverage planner
        msg = Bool()
        msg.data = True
        self.exploration_complete_pub.publish(msg)
        
        # Also tell coverage to start explicitly
        ctrl_msg = String()
        ctrl_msg.data = 'start'
        self.coverage_control_pub.publish(ctrl_msg)

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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
