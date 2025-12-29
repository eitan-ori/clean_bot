import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    pkg_share = get_package_share_directory('my_robot_slam')
    urdf_file = os.path.join(pkg_share, 'urdf', 'my_robot.urdf')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # Robot State Publisher - מפרסם את ה-TF הסטטי של הרובוט
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf_file]),

        # Lidar Node (sllidar_ros2) - יש להוריד את החבילה ולהפעיל
        # Node(
        #     package='sllidar_ros2',
        #     executable='sllidar_node',
        #     name='sllidar_node',
        #     parameters=[{'channel_type': 'serial',
        #                  'serial_port': '/dev/ttyUSB0',
        #                  'serial_baudrate': 115200,
        #                  'frame_id': 'laser_frame',
        #                  'inverted': False,
        #                  'angle_compensate': True}],
        #     output='screen'),
    ])
