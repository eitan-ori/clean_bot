import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():

    gui = LaunchConfiguration('gui')

    declare_gui_cmd = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Set to "false" to run headless.')

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='clean_bot_description' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # World file - apartment with multiple rooms
    world_file = os.path.join(get_package_share_directory('clean_bot_gazebo'), 'worlds', 'apartment.world')

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'world': world_file, 'gui': gui}.items()
             )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have one robot.
    # Spawn robot in the living room area of the apartment (center of open space)
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'my_bot',
                                   '-x', '3.0',
                                   '-y', '5.5',
                                   '-z', '0.05'],
                        output='screen')


    # Launch them all!
    return LaunchDescription([
        declare_gui_cmd,
        rsp,
        gazebo,
        spawn_entity,
    ])
