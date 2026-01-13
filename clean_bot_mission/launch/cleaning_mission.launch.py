#!/usr/bin/env python3
"""
Full Cleaning Mission Launch File

Launches the complete autonomous cleaning system:
1. Hardware drivers (from clean_bot_hardware)
2. SLAM for mapping
3. Nav2 for navigation
4. Mission controller (exploration → coverage)

Usage:
    # Full mission (explore first, then clean)
    ros2 launch clean_bot_mission cleaning_mission.launch.py

    # Skip exploration (use existing map)
    ros2 launch clean_bot_mission cleaning_mission.launch.py skip_exploration:=true

    # With custom coverage width
    ros2 launch clean_bot_mission cleaning_mission.launch.py coverage_width:=0.14
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ===================== Packages =====================
    hardware_pkg = get_package_share_directory('clean_bot_hardware')
    
    # ===================== Launch Arguments =====================
    coverage_width = LaunchConfiguration('coverage_width', default='0.14')
    skip_exploration = LaunchConfiguration('skip_exploration', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='false')
    
    # Hardware arguments (pass through to hardware launch)
    arduino_port = LaunchConfiguration('arduino_port', default='/dev/ttyACM0')
    lidar_port = LaunchConfiguration('lidar_port', default='/dev/ttyUSB0')

    return LaunchDescription([
        # ===================== Arguments =====================
        DeclareLaunchArgument('coverage_width', default_value='0.14',
                              description='Cleaning coverage width in meters'),
        DeclareLaunchArgument('skip_exploration', default_value='false',
                              description='Skip exploration and use existing map'),
        DeclareLaunchArgument('use_rviz', default_value='false',
                              description='Launch RViz for visualization'),
        DeclareLaunchArgument('arduino_port', default_value='/dev/ttyACM0',
                              description='Arduino serial port'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0',
                              description='Lidar serial port'),

        # ===================== Hardware + SLAM + Nav2 =====================
        # Include the main robot bringup (handles all hardware, SLAM, Nav2)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(hardware_pkg, 'launch', 'robot_bringup.launch.py')
            ),
            launch_arguments={
                'arduino_port': arduino_port,
                'lidar_port': lidar_port,
                'use_slam': 'true',
                'use_nav2': 'true',
            }.items()
        ),

        # ===================== Mission Nodes =====================
        # NOTE: frontier_explorer and adaptive_coverage are created internally by full_mission node
        # Do NOT launch them separately to avoid duplicate nodes and conflicting auto-start behavior
        # NOTE: cleaning_switch is already launched by robot_bringup.launch.py, do not launch again

        # Full Mission Controller - receives commands from Telegram bridge

        Node(
            package='clean_bot_mission',
            executable='full_mission',
            name='full_mission_controller',
            output='screen',
            parameters=[{
                'coverage_width': coverage_width,
                'auto_start': False,  # Wait for Telegram command
                'return_home_after': True,
            }]
        ),


    ])
