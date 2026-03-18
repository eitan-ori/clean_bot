import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Paths to packages
    hardware_pkg = get_package_share_directory('clean_bot_hardware')
    description_pkg = get_package_share_directory('clean_bot_description')

    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # 1. Launch Sensors (Lidar + IMU)
    # Disable IMU odom because we use rf2o
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hardware_pkg, 'launch', 'sensors.launch.py')
        ),
        launch_arguments={'publish_odom': 'false'}.items()
    )

    # 2.1 Joint State Publisher (Fixes RViz wheel errors)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )

    # 3. Launch Cartographer
    cartographer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hardware_pkg, 'launch', 'cartographer.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulated clock (always false for physical robot)'),
        sensors_launch,
        joint_state_publisher_node,
        cartographer_launch,
    ])
