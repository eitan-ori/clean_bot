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
# - use_sim_time: false (Always false for physical robot).
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
from launch.conditions import IfCondition, UnlessCondition
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
    lidar_port = LaunchConfiguration('lidar_port', default='/dev/lidar')
    i2c_bus = LaunchConfiguration('i2c_bus', default='1')
    
    # Robot parameters (for calibration)
    wheel_radius = LaunchConfiguration('wheel_radius', default='0.0335')
    wheel_separation = LaunchConfiguration('wheel_separation', default='0.20')
    
    # Enable/disable components
    use_nav2 = LaunchConfiguration('use_nav2', default='true')
    use_slam = LaunchConfiguration('use_slam', default='true')
    use_emergency_stop = LaunchConfiguration('use_emergency_stop', default='true')
    use_rf2o = LaunchConfiguration('use_rf2o', default='false')
    use_imu = LaunchConfiguration('use_imu', default='false')
    velocity_factor = LaunchConfiguration('velocity_factor', default='1.0')
    
    # ==================== Robot Description ====================
    xacro_file = os.path.join(description_pkg, 'urdf', 'robot.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # ==================== Config Files ====================
    slam_config = os.path.join(hardware_pkg, 'config', 'mapper_params_online_async.yaml')
    
    return LaunchDescription([
        # ==================== Declare Arguments ====================
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('arduino_port', default_value='/dev/ttyACM0',
                              description='Serial port for Arduino'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/lidar',
                              description='Serial port for RPLidar'),
        DeclareLaunchArgument('i2c_bus', default_value='1',
                              description='I2C bus number for IMU'),
        DeclareLaunchArgument('wheel_radius', default_value='0.0335',
                              description='Wheel radius in meters'),
        DeclareLaunchArgument('wheel_separation', default_value='0.20',
                              description='Distance between wheels in meters'),
        DeclareLaunchArgument('use_nav2', default_value='true',
                              description='Launch Nav2 navigation stack'),
        DeclareLaunchArgument('use_slam', default_value='true',
                              description='Launch SLAM Toolbox for mapping'),
        DeclareLaunchArgument('use_emergency_stop', default_value='true',
                              description='Launch emergency stop safety node'),
        DeclareLaunchArgument('use_rf2o', default_value='false',
                      description='Launch rf2o_laser_odometry (external odom). Default is false; Cartographer provides odom->base_link TF.'),
        DeclareLaunchArgument(
            'use_imu',
            default_value='false',
            description='Whether to launch IMU nodes (requires real IMU hardware)'),
        DeclareLaunchArgument('velocity_factor', default_value='1.0',
                              description='Velocity multiplier (2.0 = twice as fast)'),

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
        # Handles: Motors and Ultrasonic sensor
        # NOTE: No wheel encoders - odometry is provided by SLAM (Cartographer)
        Node(
            package='clean_bot_hardware',
            executable='arduino_driver',
            name='arduino_driver',
            output='screen',
            respawn=True,
            respawn_delay=10.0,
            parameters=[{
                'serial_port': arduino_port,
                'baud_rate': 57600,
                'wheel_separation': wheel_separation,
                'velocity_factor': velocity_factor,
            }]
        ),

        # ==================== RPLidar A1 ====================
        # Note: If LiDAR fails, SLAM will not be able to provide odom/pose updates.
        # Common issues: wrong port, no power, permissions (sudo chmod 666 /dev/ttyUSB0)
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
                'inverted': True,  # LiDAR mounted 180 degrees rotated
                'angle_compensate': True,
                'scan_mode': '',  # Auto-detect scan mode
                'scan_frequency': 7.0,  # Reduce CPU load (SLAM + costmaps) on Pi-class CPUs
                'force_scan': True,  # Force scan bypasses motor speed check (fixes laser timeout)
            }],
            # Publish raw scans at full rate; throttle node limits rate for consumers
            remappings=[('/scan', '/scan_raw')],
            respawn=False,
        ),
        # ==================== Scan Throttle ====================
        # Pi 4 can't process 10Hz scans across all consumers (SLAM + 2 costmaps).
        # Throttle to 5Hz for consumers while SLAM still uses /scan_raw.
        Node(
            package='clean_bot_hardware',
            executable='scan_throttle',
            name='scan_throttle',
            output='screen',
            parameters=[{'rate': 5.0}],
        ),

        # ==================== Laser Odometry (rf2o) ====================
        # Optional external odometry.
        # If disabled (default), Cartographer publishes TF(odom→base_link).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(hardware_pkg, 'launch', 'odom.launch.py')
            ),
            condition=IfCondition(use_rf2o),
        ),

        # ==================== /odom Publisher from TF ====================
        # Nav2 and monitors expect a nav_msgs/Odometry on /odom.
        # When rf2o is disabled, publish /odom from TF(odom→base_link) instead.
        Node(
            package='clean_bot_hardware',
            executable='tf_odom_publisher',
            name='tf_odom_publisher',
            output='screen',
            condition=UnlessCondition(use_rf2o),
            parameters=[{
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'odom_topic': '/odom',
                'publish_rate': 20.0,
            }],
        ),

        # ==================== IMU Publisher ====================
        Node(
            package='clean_bot_hardware',
            executable='imu_publisher',
            name='imu_publisher',
            output='screen',
            condition=IfCondition(use_imu),
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
                'publish_frame': 'base_link',
                'sensor_offset_x': 0.20,
                'sensor_offset_y': 0.0,
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
            condition=IfCondition(use_emergency_stop),
            parameters=[{
                'emergency_stop_distance': 0.10,   # 10cm - full stop
                'slow_down_distance': 0.30,        # 30cm - reduce speed
                'slow_down_factor': 0.3,           # 30% of original speed
                'reverse_allowed': True,           # Allow backing up
            }]
        ),

        # NOTE: Cleaning control is handled by arduino_driver via arduino_command topic
        # The servo/relay commands are sent to Arduino which handles the actual hardware

        # ==================== IMU Madgwick Filter ====================
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            output='screen',
            condition=IfCondition(use_imu),
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
        # ==================== Cartographer SLAM ====================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(hardware_pkg, 'launch', 'cartographer.launch.py')
            ),
            condition=IfCondition(use_slam),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        # ==================== Nav2 Navigation Stack ====================
        # (Includes: Planner, Controller, Costmap, BT Navigator, etc.)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(hardware_pkg, 'launch', 'nav2_navigation.launch.py')
            ),
            condition=IfCondition(use_nav2),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': os.path.join(hardware_pkg, 'config', 'nav2_params.yaml'),
                'use_composition': 'False',
            }.items()
        ),
    ])
