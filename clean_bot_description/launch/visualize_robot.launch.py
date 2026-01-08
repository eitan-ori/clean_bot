'''
This launch file is used for standalone visualization of the robot model in RViz.
It combines the robot state publisher, joint state publisher, and an RViz instance.

Components:
- robot_state_publisher: Converts Xacro/URDF to the '/robot_description' topic.
- joint_state_publisher: Provides dummy state data for non-fixed joints (e.g., wheels).
- RViz2: Opens the visualization window with a pre-defined configuration.

Parameters:
- None.

Main Files Used:
- urdf/robot.urdf.xacro: The source robot model.
- config/robot_visualization.rviz: The pre-saved layout for RViz dashboard.

Assumptions:
- The user has RViz2 installed and graphic acceleration is available.
- This is primarily used for verifying the model's appearance and joint hierarchy.
'''

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'clean_bot_description'
    pkg_share = get_package_share_directory(package_name)
    
    # Process xacro file
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    robot_description_config = Command(['xacro ', xacro_file])
    
    # RViz config file
    rviz_config_file = os.path.join(pkg_share, 'config', 'robot_visualization.rviz')

    # Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_config}]
    )

    # Joint State Publisher (publishes static joints)
    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen'
    )

    # RViz
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher,
        node_rviz
    ])
