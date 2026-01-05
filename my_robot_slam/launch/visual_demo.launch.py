from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_slam')

    # Include the main docking simulation launch file
    docking_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'docking_simulation.launch.py')
        )
    )

    # SLAM Toolbox (Async)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'online_async_launch.py')
        )
    )

    # Demo Mission Controller
    demo_mission_node = Node(
        package='my_robot_slam',
        executable='demo_mission.py',
        name='demo_mission',
        output='screen'
    )

    # RViz Configuration
    rviz_config_file = os.path.join(pkg_share, 'config', 'docking_demo.rviz')

    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        docking_sim_launch,
        slam_launch,
        demo_mission_node,
        rviz_node
    ])
