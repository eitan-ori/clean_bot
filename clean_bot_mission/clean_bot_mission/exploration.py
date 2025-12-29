#!/usr/bin/env python3
"""
Clean Bot Exploration Mission

This node executes an autonomous exploration mission to map an unknown
multi-room apartment using Nav2 navigation and SLAM Toolbox.

The robot follows waypoints through different rooms:
- Living Room
- Kitchen
- Bedroom 1
- Bedroom 2
- Bathroom/Hallway

Author: Clean Bot Team
"""

import time
import math
import rclpy
from rclpy.duration import Duration
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
    """Main entry point for the exploration mission."""
    rclpy.init()

    navigator = BasicNavigator()
    
    print('=' * 60)
    print('🏠 Clean Bot Apartment Exploration Mission')
    print('=' * 60)

    # Robot starts in the living room at (3.0, 5.5)
    initial_pose = create_pose(navigator, 3.0, 5.5, 0.0)
    
    print('⏳ Waiting for Nav2 bt_navigator to become active...')
    
    # Wait for bt_navigator only (SLAM doesn't use lifecycle interface)
    navigator._waitForNodeToActivate('bt_navigator')
    
    print('✅ Nav2 is active!')

    # Set the initial pose for the robot
    navigator.setInitialPose(initial_pose)
    print(f'📍 Initial pose set at living room (3.0, 5.5)')

    # Wait for SLAM localization to stabilize
    time.sleep(3.0)

    # Define exploration waypoints - using smaller incremental steps 
    # to allow SLAM to build the map progressively as the robot moves.
    # Robot starts at (3.0, 5.5) in living room
    # Doorways: Kitchen (x=3.5, y=3), Bedrooms (x=6, y=5), Bedroom2 (x=8, y=5)
    exploration_waypoints = [
        # Phase 1: Explore Living Room area - small steps from start
        {'name': 'Living Room Center', 'x': 3.5, 'y': 5.5, 'yaw': 0.0},
        {'name': 'Living Room East', 'x': 4.5, 'y': 5.5, 'yaw': 0.0},
        {'name': 'Living Room North', 'x': 4.0, 'y': 7.0, 'yaw': 1.57},
        {'name': 'Living Room West', 'x': 2.0, 'y': 6.0, 'yaw': 3.14},
        
        # Phase 2: Go to Kitchen through doorway (at x=3.5, y=3)
        {'name': 'Approach Kitchen Door', 'x': 3.5, 'y': 4.0, 'yaw': -1.57},
        {'name': 'Kitchen Door', 'x': 3.5, 'y': 3.5, 'yaw': -1.57},
        {'name': 'Kitchen Center', 'x': 3.5, 'y': 2.0, 'yaw': -1.57},
        {'name': 'Kitchen West', 'x': 1.5, 'y': 1.5, 'yaw': 3.14},
        {'name': 'Kitchen East', 'x': 5.0, 'y': 1.5, 'yaw': 0.0},
        
        # Phase 3: Return to living room  
        {'name': 'Back Through Kitchen Door', 'x': 3.5, 'y': 3.5, 'yaw': 1.57},
        {'name': 'Back to Living Room', 'x': 3.5, 'y': 5.0, 'yaw': 1.57},
        
        # Phase 4: Go to Bedrooms through doorway (at x=6, y=5)
        {'name': 'Approach Bedroom Door', 'x': 5.5, 'y': 5.0, 'yaw': 0.0},
        {'name': 'Bedroom Hallway', 'x': 6.5, 'y': 5.0, 'yaw': 0.0},
        {'name': 'Bedroom 1', 'x': 7.0, 'y': 6.5, 'yaw': 1.57},
        
        # Phase 5: Go to Bedroom 2 through doorway (at x=8, y=5)
        {'name': 'Approach Bedroom 2', 'x': 8.5, 'y': 5.0, 'yaw': 0.0},
        {'name': 'Bedroom 2', 'x': 9.0, 'y': 6.5, 'yaw': 1.57},
        
        # Phase 6: Return to start
        {'name': 'Return to Hallway', 'x': 6.5, 'y': 5.0, 'yaw': 3.14},
        {'name': 'Return Home', 'x': 3.0, 'y': 5.5, 'yaw': 3.14},
    ]

    print(f'\n📋 Exploration plan: {len(exploration_waypoints)} waypoints')
    print('-' * 60)

    # Execute exploration
    successful_waypoints = 0
    failed_waypoints = 0

    for i, wp in enumerate(exploration_waypoints, 1):
        print(f'\n🚀 [{i}/{len(exploration_waypoints)}] Navigating to: {wp["name"]}')
        print(f'   Position: ({wp["x"]:.1f}, {wp["y"]:.1f}), Yaw: {math.degrees(wp["yaw"]):.0f}°')
        
        goal = create_pose(navigator, wp['x'], wp['y'], wp['yaw'])
        navigator.goToPose(goal)

        # Monitor navigation progress with timeout
        start_time = time.time()
        timeout = 45.0  # 45 second timeout per waypoint
        
        while not navigator.isTaskComplete():
            if time.time() - start_time > timeout:
                print(f'\n   ⏱️  Timeout - skipping waypoint')
                navigator.cancelTask()
                break
                
            feedback = navigator.getFeedback()
            if feedback:
                remaining = feedback.distance_remaining
                if remaining > 0.1:  # Only show if meaningful
                    print(f'   📏 Distance remaining: {remaining:.2f}m', end='\r')
            time.sleep(0.5)

        result = navigator.getResult()
        
        if result == TaskResult.SUCCEEDED:
            print(f'   ✅ Reached {wp["name"]}!')
            successful_waypoints += 1
            # Give SLAM time to process new area
            print(f'   🗺️  Updating map...')
            time.sleep(3.0)
        elif result == TaskResult.CANCELED:
            print(f'   ⚠️  Navigation to {wp["name"]} was canceled/timed out')
            failed_waypoints += 1
            time.sleep(1.0)
        elif result == TaskResult.FAILED:
            print(f'   ❌ Failed to reach {wp["name"]} - trying next waypoint')
            failed_waypoints += 1
            time.sleep(1.0)
        else:
            print(f'   ❓ Unknown result for {wp["name"]}')
            failed_waypoints += 1

    # Mission summary
    print('\n' + '=' * 60)
    print('🏁 EXPLORATION MISSION COMPLETE')
    print('=' * 60)
    print(f'✅ Successful waypoints: {successful_waypoints}/{len(exploration_waypoints)}')
    print(f'❌ Failed waypoints: {failed_waypoints}/{len(exploration_waypoints)}')
    
    if successful_waypoints == len(exploration_waypoints):
        print('\n🎉 Perfect exploration! All rooms mapped successfully!')
    elif successful_waypoints > len(exploration_waypoints) / 2:
        print('\n👍 Good exploration coverage achieved!')
    else:
        print('\n⚠️  Partial exploration - some areas may need re-mapping')

    print('\n💡 The generated map can be saved with:')
    print('   ros2 run nav2_map_server map_saver_cli -f ~/apartment_map')
    
    navigator.lifecycleShutdown()
    rclpy.shutdown()
    
    exit(0)


if __name__ == '__main__':
    main()
