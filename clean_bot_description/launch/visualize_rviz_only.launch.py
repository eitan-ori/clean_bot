'''
This launch file starts ONLY the RViz2 visualization tool. It does not process
the URDF or start state publishers.

Parameters:
- None.

Main Files Used:
- config/robot_visualization.rviz: The configuration file for the RViz layout.

Assumptions:
- The robot's state and description are already being published by another
  running node (e.g., in a simulation or on real hardware).
'''

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
