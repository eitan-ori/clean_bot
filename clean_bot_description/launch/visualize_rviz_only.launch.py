import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'clean_bot_description'
    pkg_share = get_package_share_directory(package_name)
    
    # RViz config file
    rviz_config_file = os.path.join(pkg_share, 'config', 'robot_visualization.rviz')

    # RViz
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        node_rviz
    ])
