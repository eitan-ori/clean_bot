#!/usr/bin/env python3
"""
###############################################################################
# FILE DESCRIPTION:
# This is the master bringup launch file for the Clean Bot robot. It initializes
# all hardware drivers, perception filters, and the navigation/mapping stack.
#
# MAIN FUNCTIONS:
# 1. Launches Hardware Drivers: Arduino (motors/encoders), RPLidar, and IMU.
# 2. Starts Robot State Publisher: Broadcasts URDF and static TF transforms.
# 3. Perception: Optional SLAM Toolbox for real-time mapping.
# 4. Navigation: Optional Nav2 stack for autonomous movement.
# 5. Sensor Fusion: Configures EKF (robot_localization) if available.
#
# PARAMETERS & VALUES:
# - use_sim_time: false (Set to true only when running in Gazebo).
# - arduino_port: /dev/ttyUSB0 (Primary motor/sensor interface).
# - lidar_port: /dev/ttyUSB1 (Laser scanner interface).
# - i2c_bus: 1 (I2C interface for the IMU).
# - wheel_radius: 0.034 m (Calibrated wheel measurement).
# - wheel_separation: 0.20 m (Calibrated width measurement).
# - use_nav2: true (Whether to start the navigation stack).
# - use_slam: true (Whether to start mapping).
#
# ASSUMPTIONS:
# - All physical hardware is connected to the corresponding USB/I2C ports.
# - The 'clean_bot_description' package is built and available.
# - Necessary ROS 2 components (SLAM Toolbox, Nav2) are installed on the host.
###############################################################################
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ==================== Package Paths ====================
    hardware_pkg = get_package_share_directory('clean_bot_hardware')
    description_pkg = get_package_share_directory('clean_bot_description')
    
    # ==================== Launch Arguments ====================
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Hardware ports
    arduino_port = LaunchConfiguration('arduino_port', default='/dev/ttyACM0')
    lidar_port = LaunchConfiguration('lidar_port', default='/dev/ttyUSB0')
    i2c_bus = LaunchConfiguration('i2c_bus', default='1')
    
    # Robot parameters (for calibration)
    wheel_radius = LaunchConfiguration('wheel_radius', default='0.0335')
    wheel_separation = LaunchConfiguration('wheel_separation', default='0.20')
    
    # Enable/disable components
    use_nav2 = LaunchConfiguration('use_nav2', default='true')
    use_slam = LaunchConfiguration('use_slam', default='true')
    
    # ==================== Robot Description ====================
    xacro_file = os.path.join(description_pkg, 'urdf', 'robot.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # ==================== Config Files ====================
    slam_config = os.path.join(hardware_pkg, 'config', 'mapper_params_online_async.yaml')
    ekf_config = os.path.join(hardware_pkg, 'config', 'ekf.yaml')
    
    return LaunchDescription([
        # ==================== Declare Arguments ====================
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('arduino_port', default_value='/dev/ttyUSB0',
                              description='Serial port for Arduino'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB1',
                              description='Serial port for RPLidar'),
        DeclareLaunchArgument('i2c_bus', default_value='1',
                              description='I2C bus number for IMU'),
        DeclareLaunchArgument('wheel_radius', default_value='0.034',
                              description='Wheel radius in meters'),
        DeclareLaunchArgument('wheel_separation', default_value='0.20',
                              description='Distance between wheels in meters'),
        DeclareLaunchArgument('use_nav2', default_value='true',
                              description='Launch Nav2 navigation stack'),
        DeclareLaunchArgument('use_slam', default_value='true',
                              description='Launch SLAM Toolbox for mapping'),

        # ==================== Robot State Publisher ====================
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time
            }]
        ),

        # ==================== Arduino Driver ====================
        # Handles: Motors, Encoders, Ultrasonic
        # Note: Wheel odometry published but NOT used - laser odom is primary
        Node(
            package='clean_bot_hardware',
            executable='arduino_driver',
            name='arduino_driver',
            output='screen',
            parameters=[{
                'serial_port': arduino_port,
                'baud_rate': 57600,
                'wheel_radius': wheel_radius,
                'wheel_separation': wheel_separation,
                'ticks_per_revolution': 1320,  # GB37-131: 11 CPR * 120 gear ratio
                'publish_tf': False,  # EKF publishes odom->base_link
                'odom_frame_id': 'odom',
                'base_frame_id': 'base_link',
            }]
        ),

        # ==================== RPLidar A1 ====================
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': lidar_port,
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Sensitivity',
            }]
        ),

        # ==================== IMU Publisher ====================
        Node(
            package='clean_bot_hardware',
            executable='imu_publisher',
            name='imu_publisher',
            output='screen',
            parameters=[{
                'i2c_bus': i2c_bus,
                'frame_id': 'imu_link',
                'publish_rate': 50.0,
            }]
        ),

        # ==================== Low Obstacle Detector ====================
        # Converts ultrasonic readings to PointCloud2 for costmap
        # Detects obstacles at 3cm height (cables, shoes, pet bowls, etc.)
        Node(
            package='clean_bot_hardware',
            executable='low_obstacle_detector',
            name='low_obstacle_detector',
            output='screen',
            parameters=[{
                'ultrasonic_frame': 'ultrasonic_link',
                'min_obstacle_distance': 0.05,   # 5cm
                'max_obstacle_distance': 0.50,   # 50cm
                'obstacle_height': 0.03,         # 3cm sensor height
                'obstacle_persistence': 2.0,     # Keep obstacle for 2 seconds
            }]
        ),

        # ==================== Emergency Stop Controller ====================
        # Safety layer - stops robot when ultrasonic detects very close obstacle
        # Acts as velocity mux between Nav2 and motors
        Node(
            package='clean_bot_hardware',
            executable='emergency_stop',
            name='emergency_stop_controller',
            output='screen',
            parameters=[{
                'emergency_stop_distance': 0.10,   # 10cm - full stop
                'slow_down_distance': 0.30,        # 30cm - reduce speed
                'slow_down_factor': 0.3,           # 30% of original speed
                'reverse_allowed': True,           # Allow backing up
            }]
        ),

        # ==================== IMU Madgwick Filter ====================
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            output='screen',
            parameters=[{
                'use_mag': False,  # Disable magnetometer (not calibrated)
                'publish_tf': False,
                'world_frame': 'enu',
                'gain': 0.1,
            }],
            remappings=[
                ('imu/data_raw', 'imu/data_raw'),
                ('imu/mag', 'imu/mag'),
                ('imu/data', 'imu/data'),
            ]
        ),

        # ==================== Laser Odometry (ICP Scan Matching) ====================
        # Uses rtabmap's ICP odometry - much better than wheel encoders!
        Node(
            package='rtabmap_odom',
            executable='icp_odometry',
            name='icp_odometry',
            output='screen',
            parameters=[{
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'publish_tf': False,  # EKF will publish tf
                'wait_for_transform': 0.2,
                'Icp/VoxelSize': '0.0',  # No voxel filtering for 2D
                'Icp/MaxCorrespondenceDistance': '0.15',
                'Icp/Iterations': '30',
                'Icp/PointToPlane': 'false',  # Point-to-point for 2D
                'Odom/Strategy': '0',  # ICP
                'Odom/GuessMotion': 'true',
                'Odom/ResetCountdown': '0',
                'Odom/ScanKeyFrameThr': '0.7',
            }],
            remappings=[
                ('scan', '/scan'),
                ('odom', '/laser_odom'),
            ]
        ),

        # ==================== Robot Localization (EKF) ====================
        # Fuses LASER odometry + IMU angular velocity
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config, {'use_sim_time': use_sim_time}]
        ),

        # ==================== SLAM Toolbox ====================
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            condition=IfCondition(use_slam),
            parameters=[slam_config, {'use_sim_time': use_sim_time}]
        ),

        # ==================== Nav2 Navigation Stack ====================
        # (Includes: Planner, Controller, Costmap, BT Navigator, etc.)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('nav2_bringup'),
                    'launch', 'navigation_launch.py'
                )
            ),
            condition=IfCondition(use_nav2),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': os.path.join(hardware_pkg, 'config', 'nav2_params.yaml'),
            }.items()
        ),
    ])
