import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_slam')
    nav2_dir = get_package_share_directory('nav2_bringup')

    # Configuration Files
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')
    slam_config = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')
    nav2_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # 1. Hardware Drivers & Robot State
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'bringup.launch.py')),
            launch_arguments={'use_sim_time': use_sim_time}.items()
        ),

        Node(
            package='my_robot_slam',
            executable='arduino_bridge.py',
            name='arduino_bridge',
            output='screen'
        ),

        Node(
            package='my_robot_slam',
            executable='imu_driver.py',
            name='imu_driver',
            output='screen'
        ),

        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{'channel_type': 'serial',
                         # Check your port!
                         'serial_port': '/dev/ttyUSB1',
                         'serial_baudrate': 115200,
                         'frame_id': 'laser_frame',
                         'inverted': False,
                         'angle_compensate': True}],
            output='screen'),

        # 2. Sensor Fusion (EKF)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config, {'use_sim_time': use_sim_time}]
        ),

        # 3. SLAM (Mapping)
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_config, {'use_sim_time': use_sim_time}]
        ),

        # 4. Navigation (Nav2)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_dir, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': nav2_params
            }.items()
        ),

        # 5. Mission Control (Coverage Planner)
        Node(
            package='my_robot_slam',
            executable='coverage_planner.py',
            name='coverage_planner',
            output='screen',
            parameters=[{'robot_width': 0.3}]
        ),
    ])
