#!/usr/bin/env python3
"""
###############################################################################
# FILE DESCRIPTION:
# This launch file initializes the physical IMU sensor and optionally starts
# an orientation filter (Madgwick) to process the raw sensor data.
#
# MAIN FUNCTIONS:
# 1. Starts the 'imu_publisher_node' to read data from the I2C bus.
# 2. Optionally starts 'imu_filter_madgwick_node' to calculate the robot's 
#    absolute orientation (Heading/Pitch/Roll) from raw IMU and Magnetometer data.
#
# PARAMETERS & VALUES:
# - i2c_bus: 1 (The I2C bus number).
# - frame_id: imu_link (TF frame associated with the IMU).
# - publish_rate: 50.0 Hz (Sampling frequency).
# - use_madgwick: true (Whether to run the fusion filter).
# - use_mag: true (Whether to include the magnetometer in the fusion).
#
# ASSUMPTIONS:
# - The 'imu_filter_madgwick' ROS 2 package is installed for orientation fusion.
# - The sensor is mounted flat relative to the robot's base frame.
###############################################################################
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Arguments
    i2c_bus = LaunchConfiguration('i2c_bus', default='1')
    frame_id = LaunchConfiguration('frame_id', default='imu_link')
    publish_rate = LaunchConfiguration('publish_rate', default='50.0')
    use_madgwick = LaunchConfiguration('use_madgwick', default='true')
    use_mag = LaunchConfiguration('use_mag', default='true')
    
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'i2c_bus',
            default_value='1',
            description='I2C bus number'),
        DeclareLaunchArgument(
            'frame_id',
            default_value='imu_link',
            description='Frame ID for IMU'),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='50.0',
            description='Publish rate in Hz'),
        DeclareLaunchArgument(
            'use_madgwick',
            default_value='true',
            description='Use Madgwick filter'),
        DeclareLaunchArgument(
            'use_mag',
            default_value='true',
            description='Use magnetometer'),
        
        # IMU Publisher
        Node(
            package='clean_bot_hardware',
            executable='imu_publisher',
            name='imu_publisher_node',
            parameters=[{
                'i2c_bus': i2c_bus,
                'frame_id': frame_id,
                'publish_rate': publish_rate,
            }],
            output='screen'
        ),
        
        # Madgwick Filter (optional)
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            condition=IfCondition(use_madgwick),
            parameters=[{
                'use_mag': use_mag,
                'publish_tf': False,
                'world_frame': 'enu',
                'fixed_frame': frame_id,
            }],
            remappings=[
                ('/imu/data_raw', '/imu/data_raw'),
                ('/imu/mag', '/imu/mag'),
            ],
            output='screen'
        ),
    ])
