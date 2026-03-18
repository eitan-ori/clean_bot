#!/usr/bin/env python3
"""
Scan Room Launch File
=====================
Single launch file that:
  1. Brings up ALL robot hardware (LiDAR, Arduino, SLAM, Nav2)
  2. Auto-starts the scan/exploration mission after 5 seconds
  3. Runs a diagnostic monitor that prints ONLY relevant events

Usage:
    ros2 launch clean_bot_mission scan_room.launch.py

    # Override ports if needed:
    ros2 launch clean_bot_mission scan_room.launch.py lidar_port:=/dev/ttyUSB1

Output shows:
  ✅ MAP RECEIVED, ✅ ODOM RECEIVED, ✅ LIDAR READY
  🔍 Exploration started / frontier goals / velocity flow
  ⚡ cmd_vel_nav → cmd_vel → cmd_vel_safe  (the full chain to motors)
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    SetEnvironmentVariable,
    LogInfo,
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    hardware_pkg = get_package_share_directory('clean_bot_hardware')

    arduino_port = LaunchConfiguration('arduino_port', default='/dev/ttyACM0')
    lidar_port = LaunchConfiguration('lidar_port', default='/dev/lidar')
    velocity_factor = LaunchConfiguration('velocity_factor', default='1.0')

    return LaunchDescription([
        # ── Arguments ──
        DeclareLaunchArgument('arduino_port', default_value='/dev/ttyACM0',
                              description='Arduino serial port'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/lidar',
                              description='LiDAR serial port (use /dev/lidar if udev rule set)'),
        DeclareLaunchArgument('velocity_factor', default_value='1.0',
                              description='Velocity multiplier (2.0 = twice as fast)'),

        # Reduce console noise from Nav2 and other infrastructure
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT',
                               '[{name}] {message}'),

        LogInfo(msg='\n'
                '╔══════════════════════════════════════════════════╗\n'
                '║         🤖  SCAN ROOM - AUTO START  🤖         ║\n'
                '║  Hardware → SLAM → Nav2 → Explore frontiers     ║\n'
                '║  Scan will auto-start 5s after launch           ║\n'
                '╚══════════════════════════════════════════════════╝\n'),

        # ── 1. Robot hardware + SLAM + Nav2 ──
        # emergency_stop disabled for debugging — arduino_driver gets cmd_vel_nav directly
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(hardware_pkg, 'launch', 'robot_bringup.launch.py')
            ),
            launch_arguments={
                'arduino_port': arduino_port,
                'lidar_port': lidar_port,
                'use_slam': 'true',
                'use_nav2': 'true',
                'use_emergency_stop': 'false',
                'velocity_factor': velocity_factor,
            }.items()
        ),

        # ── 2. Full Mission Controller (auto_start=true) ──
        Node(
            package='clean_bot_mission',
            executable='full_mission',
            name='full_mission_controller',
            output='log',  # suppress to log file — monitor prints relevant info
            arguments=['--ros-args', '--log-level', 'warn'],
            parameters=[{
                'coverage_width': 0.14,
                'auto_start': True,
                'return_home_after': True,
            }],
        ),

        # ── 3. Diagnostic monitor (the ONLY node printing to screen) ──
        Node(
            package='clean_bot_mission',
            executable='scan_monitor',
            name='scan_monitor',
            output='screen',
        ),
    ])
