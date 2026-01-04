#!/usr/bin/env python3
"""
Combined launch file for all Clean Bot sensors AND Robot State Publisher:
- SLAMTEC Lidar (sllidar_ros2)
- Grove IMU 9DOF (ICM20600 + AK09918)
- IMU Madgwick Filter (optional)
- Robot State Publisher (URDF)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ==================== Launch Arguments ====================
    
    # Lidar arguments
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    lidar_frame_id = LaunchConfiguration('lidar_frame_id', default='laser')
    
    # IMU arguments
    i2c_bus = LaunchConfiguration('i2c_bus', default='1')
    imu_frame_id = LaunchConfiguration('imu_frame_id', default='imu_link')
    imu_rate = LaunchConfiguration('imu_rate', default='50.0')
    
    # Madgwick filter arguments
    use_madgwick = LaunchConfiguration('use_madgwick', default='true')
    use_mag = LaunchConfiguration('use_mag', default='true')
    
    # ==================== Robot Description ====================
    pkg_description = get_package_share_directory('clean_bot_description')
    xacro_file = os.path.join(pkg_description, 'urdf', 'robot.urdf.xacro')
    robot_description_config = Command(['xacro ', xacro_file])

    return LaunchDescription([
        # ==================== Declare Arguments ====================
        
        # Lidar
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for lidar'),
        DeclareLaunchArgument(
            'serial_baudrate',
            default_value='115200',
            description='Baudrate for lidar serial port'),
        DeclareLaunchArgument(
            'lidar_frame_id',
            default_value='laser',
            description='Frame ID for lidar'),
        
        # IMU
        DeclareLaunchArgument(
            'i2c_bus',
            default_value='1',
            description='I2C bus number for IMU'),
        DeclareLaunchArgument(
            'imu_frame_id',
            default_value='imu_link',
            description='Frame ID for IMU'),
        DeclareLaunchArgument(
            'imu_rate',
            default_value='50.0',
            description='IMU publish rate in Hz'),
        
        # Madgwick
        DeclareLaunchArgument(
            'use_madgwick',
            default_value='true',
            description='Whether to use Madgwick filter for orientation'),
        DeclareLaunchArgument(
            'use_mag',
            default_value='true',
            description='Whether to use magnetometer for yaw correction'),
        
        # ==================== Robot State Publisher ====================
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_config}]
        ),

        # ==================== Lidar Node ====================
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': serial_port,
                'serial_baudrate': serial_baudrate,
                'frame_id': lidar_frame_id,
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Sensitivity',
            }],
            output='screen'
        ),
        
        # ==================== IMU Publisher Node ====================
        Node(
            package='clean_bot_hardware',
            executable='imu_publisher',
            name='imu_publisher_node',
            parameters=[{
                'i2c_bus': i2c_bus,
                'frame_id': imu_frame_id,
                'publish_rate': imu_rate,
            }],
            output='screen'
        ),
        
        # ==================== Madgwick Filter Node ====================
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            condition=IfCondition(use_madgwick),
            parameters=[{
                'use_mag': use_mag,
                'publish_tf': False,  # Disable TF, we use our own broadcaster
                'world_frame': 'enu',
                'fixed_frame': imu_frame_id,
            }],
            remappings=[
                ('/imu/data_raw', '/imu/data_raw'),
                ('/imu/mag', '/imu/mag'),
            ],
            output='screen'
        ),
        
        # ==================== IMU Odom Broadcaster ====================
        Node(
            package='clean_bot_hardware',
            executable='imu_odom_broadcaster',
            name='imu_odom_broadcaster',
            condition=IfCondition(use_madgwick),
            parameters=[{
                'imu_topic': '/imu/data',
                'parent_frame': 'odom',
                'child_frame': 'base_link',
            }],
            output='screen'
        ),
    ])
