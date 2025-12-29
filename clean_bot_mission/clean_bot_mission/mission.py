#!/usr/bin/env python3
"""
Clean Bot Mission Node

This node executes an autonomous cleaning mission using Nav2 navigation.
The robot follows a predefined square path through waypoints and returns
to the starting position (dock).

Author: Clean Bot Team
"""

import time
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def create_pose(navigator, x: float, y: float, yaw: float = 0.0) -> PoseStamped:
    """
    Create a PoseStamped message with the given position and orientation.
    
    Args:
        navigator: BasicNavigator instance for clock
        x: X position in map frame (meters)
        y: Y position in map frame (meters)
        yaw: Yaw orientation in radians (default: 0.0)
    
    Returns:
        PoseStamped message
    """
    import math
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    # Convert yaw to quaternion (rotation around Z axis)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def main():
    """Main entry point for the cleaning mission."""
    rclpy.init()

    navigator = BasicNavigator()
    
    print('=' * 50)
    print('Clean Bot Mission Starting...')
    print('=' * 50)

    # Set initial pose at the dock/starting position
    initial_pose = create_pose(navigator, 0.0, 0.0, 0.0)
    
    print('Waiting for Nav2 and SLAM Toolbox to become active...')
    
    # Wait for Nav2 navigation stack and SLAM Toolbox localization
    # Using 'slam_toolbox' as localizer since we're using SLAM, not AMCL
    navigator.waitUntilNav2Active(localizer='slam_toolbox')
    
    print('Nav2 is active!')

    # Set the initial pose for the robot
    navigator.setInitialPose(initial_pose)
    print(f'Initial pose set at (0.0, 0.0)')

    # Wait for localization to stabilize
    time.sleep(2.0)

    # Define the cleaning path waypoints (a square pattern)
    # The room is approximately 5m x 5m, so we stay within safe bounds
    waypoints = [
        (1.5, 0.0, 0.0),    # Move forward
        (1.5, 1.5, 1.57),   # Turn and move to corner
        (0.0, 1.5, 3.14),   # Move along wall
        (0.0, 0.0, -1.57),  # Return to dock
    ]
    
    goal_poses = []
    print('\nPlanned waypoints:')
    for i, (x, y, yaw) in enumerate(waypoints, 1):
        pose = create_pose(navigator, x, y, yaw)
        goal_poses.append(pose)
        print(f'  Waypoint {i}: x={x:.1f}, y={y:.1f}')

    print('\nStarting navigation through waypoints...')
    print('-' * 50)
    
    # Execute navigation through all waypoints
    navigator.goThroughPoses(goal_poses)

    # Monitor navigation progress
    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        
        if feedback and i % 5 == 0:
            eta = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
            elapsed = Duration.from_msg(feedback.navigation_time).nanoseconds / 1e9
            
            print(f'Progress: ETA={eta:.0f}s, Elapsed={elapsed:.0f}s')
            
            # Cancel if taking too long (10 minutes timeout)
            if elapsed > 600.0:
                print('Navigation timeout! Cancelling task...')
                navigator.cancelTask()
        
        time.sleep(0.1)

    # Report final result
    print('-' * 50)
    result = navigator.getResult()
    
    if result == TaskResult.SUCCEEDED:
        print('✓ Mission SUCCEEDED! Robot has completed the cleaning path.')
    elif result == TaskResult.CANCELED:
        print('✗ Mission CANCELED!')
    elif result == TaskResult.FAILED:
        print('✗ Mission FAILED! Check obstacles or path validity.')
    else:
        print('? Mission ended with unknown status.')

    print('=' * 50)
    print('Clean Bot Mission Complete')
    print('=' * 50)

    # Clean shutdown
    rclpy.shutdown()


if __name__ == '__main__':
    main()
