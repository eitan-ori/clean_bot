'''
This launch file starts the Robot State Publisher node. It processes the Xacro
description of the robot and publishes the 'robot_description' parameter,
along with the TF transforms for all static joints in the model.

Parameters:
- use_sim_time: (string/bool) If 'true', synchronized timing is taken from Gazebo/Simulation.
  Default: 'false'.

Process:
1. Locates the 'robot.urdf.xacro' file in the package's URDF directory.
2. Uses the xacro library to compile the Xacro XML into a raw URDF string.
3. Passes the URDF string to the 'robot_state_publisher' node as a parameter.

Assumptions:
- The package name is 'clean_bot_description'.
- All required Xacro sub-files are available in the urdf folder.
'''

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('clean_bot_description'))
    xacro_file = os.path.join(pkg_path,'urdf','robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )


    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        node_robot_state_publisher
    ])
