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
- "start_clean"  : Start cleaning (map-based coverage)
- "stop_clean"   : Stop cleaning
- "go_home"      : Return to home position
- "reset"        : Reset to initial WAITING_FOR_SCAN state
- "pause"        : Pause current operation
- "resume"       : Resume paused operation

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

import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient

from std_msgs.msg import Bool, String, Empty
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose

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
        
        # Physical switch control (legacy - for external nodes that might listen)
        self.clean_trigger_pub = self.create_publisher(Empty, 'clean', 10)
        self.stop_clean_trigger_pub = self.create_publisher(Empty, 'stop_clean_relay', 10)
        
        # Arduino cleaning control - publishes to mission_command for arduino_driver
        # NOTE: We use a DIFFERENT topic name to avoid infinite loop since we also subscribe to mission_command
        self.arduino_clean_pub = self.create_publisher(String, 'arduino_command', 10)

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

        # ===================== Nav2 action client =====================
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Print startup banner
        self.print_banner()
        
        # Auto-start if configured
        if self.auto_start:
            self._auto_start_timer = self.create_timer(
                3.0, self._auto_start_once, callback_group=ReentrantCallbackGroup())

    def _auto_start_once(self):
        """Auto-start exploration once, then cancel the timer."""
        if self.state == MissionState.WAITING_FOR_SCAN:
            self.get_logger().info('🚀 Auto-starting exploration...')
            self.start_exploration()
        if self._auto_start_timer is not None:
            self._auto_start_timer.cancel()
            self._auto_start_timer = None

    def print_banner(self):
        """Print startup banner."""
        W = 58  # inner width between ║ borders
        def pad(s):
            # ljust won't handle emojis perfectly but close enough for logs
            return '║' + s.ljust(W)[:W] + '║'
        self.get_logger().info('')
        self.get_logger().info('╔' + '═' * W + '╗')
        self.get_logger().info(pad('         🤖 CLEAN BOT MISSION'))
        self.get_logger().info('╠' + '═' * W + '╣')
        self.get_logger().info(pad('  Phase 1: Wait for scan command'))
        self.get_logger().info(pad('  Phase 2: Explore unknown environment'))
        self.get_logger().info(pad('  Phase 3: Wait for clean command'))
        self.get_logger().info(pad('  Phase 4: Cover all free space (clean)'))
        self.get_logger().info(pad('  Phase 5: Return home'))
        self.get_logger().info('╠' + '═' * W + '╣')
        self.get_logger().info(pad(f'  Coverage width: {self.coverage_width * 100:.0f}cm'))
        self.get_logger().info(pad(f'  Auto-start: {self.auto_start}'))
        self.get_logger().info('╠' + '═' * W + '╣')
        self.get_logger().info(pad('  Commands (publish to /mission_command):'))
        self.get_logger().info(pad('    start_scan  - Begin exploration'))
        self.get_logger().info(pad('    stop_scan   - Stop exploration'))
        self.get_logger().info(pad('    start_clean - Begin cleaning (coverage)'))
        self.get_logger().info(pad('    stop_clean  - Stop cleaning'))
        self.get_logger().info(pad('    go_home     - Return to home'))
        self.get_logger().info(pad('    reset       - Reset to initial state'))
        self.get_logger().info(pad('    pause       - Pause current operation'))
        self.get_logger().info(pad('    resume      - Resume paused operation'))
        self.get_logger().info('╚' + '═' * W + '╝')
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
        if self.state == MissionState.EXPLORING or \
           (self.state == MissionState.PAUSED and self.previous_state == MissionState.EXPLORING):
            self.get_logger().info('🛑 Stopping exploration...')
            self.stop_robot()
            self.previous_state = None
            
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
        """Handle start_clean command (map-based coverage)."""
        if self.state in [MissionState.WAITING_FOR_CLEAN, MissionState.WAITING_FOR_SCAN]:
            if self.state == MissionState.WAITING_FOR_SCAN:
                self.get_logger().info('⚠️ Starting cleaning without completing scan first')
            self.start_coverage()
        else:
            self.get_logger().warn(f'⚠️ Cannot start clean in state: {self.state}')

    def handle_stop_clean(self):
        """Handle stop_clean command."""
        if self.state == MissionState.COVERAGE or \
           (self.state == MissionState.PAUSED and self.previous_state == MissionState.COVERAGE):
            self.get_logger().info('🛑 Stopping cleaning...')
            self.stop_robot()
            self._deactivate_cleaning_hardware()
            self.previous_state = None

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
        
        # Deactivate cleaning if it was active (or paused from coverage)
        if self.state == MissionState.COVERAGE or \
           (self.state == MissionState.PAUSED and self.previous_state == MissionState.COVERAGE):
            self._deactivate_cleaning_hardware()
        
        # Stop any running operations
        ctrl_msg = String()
        ctrl_msg.data = 'stop'
        self.exploration_control_pub.publish(ctrl_msg)
        self.coverage_control_pub.publish(ctrl_msg)
        
        self.previous_state = None
        self.state = MissionState.RETURNING
        self.return_home_sequence()

    def handle_reset(self):
        """Handle reset command - go back to initial waiting state."""
        self.get_logger().info('🔄 Resetting mission...')
        self.stop_robot()
        
        # Deactivate cleaning if it was active (or paused from coverage)
        if self.state == MissionState.COVERAGE or \
           (self.state == MissionState.PAUSED and self.previous_state == MissionState.COVERAGE):
            self._deactivate_cleaning_hardware()
        
        # Stop any running operations
        ctrl_msg = String()
        ctrl_msg.data = 'stop'
        self.exploration_control_pub.publish(ctrl_msg)
        self.coverage_control_pub.publish(ctrl_msg)
        
        # Reset state
        self.state = MissionState.WAITING_FOR_SCAN
        self.previous_state = None
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
            
            # Deactivate cleaning hardware while paused to save battery and reduce noise
            if self.state == MissionState.COVERAGE:
                self._deactivate_cleaning_hardware()
            
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
            
            # Reactivate cleaning hardware if resuming coverage
            if self.previous_state == MissionState.COVERAGE:
                self._activate_cleaning_hardware()
            
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

    def _activate_cleaning_hardware(self):
        """Send cleaning start signals to Arduino (relay + servo)."""
        self.clean_trigger_pub.publish(Empty())
        arduino_msg = String()
        arduino_msg.data = 'start_clean'
        self.arduino_clean_pub.publish(arduino_msg)
        self.get_logger().info('🔌 Cleaning hardware activated')

    def _deactivate_cleaning_hardware(self):
        """Send cleaning stop signals to Arduino (relay + servo)."""
        self.stop_clean_trigger_pub.publish(Empty())
        arduino_msg = String()
        arduino_msg.data = 'stop_clean'
        self.arduino_clean_pub.publish(arduino_msg)
        self.get_logger().info('🔌 Cleaning hardware deactivated')

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
            self.stop_robot()
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
        self._activate_cleaning_hardware()

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
            self._deactivate_cleaning_hardware()
            self.get_logger().info('')
            
            if self.return_home:
                self.state = MissionState.RETURNING
                self.return_home_sequence()
            else:
                self.finish_mission()

    def return_home_sequence(self):
        """Navigate back to home position using Nav2."""
        self.get_logger().info(f'🏠 Returning to home position ({self.home_x}, {self.home_y})...')

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(self.home_x)
        goal_msg.pose.pose.position.y = float(self.home_y)
        goal_msg.pose.pose.orientation.w = 1.0

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('⚠️ Nav2 action server not available, finishing mission anyway')
            self.finish_mission()
            return

        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self._nav_goal_response_callback)

    def _nav_goal_response_callback(self, future):
        """Handle Nav2 goal acceptance/rejection."""
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().warn(f'⚠️ Nav2 goal send failed: {e}')
            self.finish_mission()
            return
        if not goal_handle.accepted:
            self.get_logger().warn('⚠️ Nav2 rejected go-home goal, finishing mission anyway')
            self.finish_mission()
            return
        self.get_logger().info('🏠 Nav2 accepted go-home goal...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_callback)

    def _nav_result_callback(self, future):
        """Handle Nav2 navigation result."""
        try:
            result = future.result()
            self.get_logger().info('🏠 Navigation to home complete')
        except Exception as e:
            self.get_logger().warn(f'⚠️ Navigation to home failed: {e}')
        self.finish_mission()

    def finish_mission(self):
        """Mission complete."""
        self.state = MissionState.COMPLETE
        
        elapsed = time.time() - self.start_time if self.start_time else 0
        
        # Stop robot
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        
        W = 58
        def pad(s):
            return '║' + s.ljust(W)[:W] + '║'
        self.get_logger().info('')
        self.get_logger().info('╔' + '═' * W + '╗')
        self.get_logger().info(pad('            🎉 MISSION COMPLETE!'))
        self.get_logger().info('╠' + '═' * W + '╣')
        self.get_logger().info(pad(f'  Total time: {elapsed / 60:.1f} minutes'))
        self.get_logger().info('╚' + '═' * W + '╝')
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
        mission_controller.stop_robot()
        mission_controller._deactivate_cleaning_hardware()
    finally:
        executor.shutdown()
        mission_controller.destroy_node()
        explorer.destroy_node()
        coverage_planner.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
