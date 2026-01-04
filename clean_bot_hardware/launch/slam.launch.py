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
    slam_pkg = get_package_share_directory('slam_toolbox')

    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # 1. Launch Sensors (Lidar + IMU)
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hardware_pkg, 'launch', 'sensors.launch.py')
        )
    )

    # 2. Launch Robot State Publisher (Already included in sensors.launch.py)
    # rsp_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(description_pkg, 'launch', 'rsp.launch.py')
    #     ),
    #     launch_arguments={'use_sim_time': use_sim_time}.items()
    # )

    # 3. Launch SLAM Toolbox
    slam_params_file = os.path.join(hardware_pkg, 'config', 'mapper_params_online_async.yaml')
    
    start_async_slam_toolbox_node = Node(
        parameters=[
          slam_params_file,
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
    )

    # 4. Launch RViz (Disabled for headless Pi)
    # rviz_config = os.path.join(hardware_pkg, 'config', 'rplidar_rviz.rviz')
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', rviz_config],
    #     output='screen'
    # )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation/Gazebo clock'),
        
        sensors_launch,
        # rsp_launch,
        start_async_slam_toolbox_node,
        # rviz_node
    ])
