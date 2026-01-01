"""Launch RViz with RPLidar configuration."""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for RViz with lidar config."""
    
    pkg_dir = get_package_share_directory('clean_bot_hardware')
    rviz_config = os.path.join(pkg_dir, 'config', 'rplidar_rviz.rviz')
    
    return LaunchDescription([
        # static transform: map -> imu_link (identity)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_map_imu',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'imu_link'],
            output='screen'
        ),
        # static transform: map -> laser (identity)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_map_laser',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'laser'],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),
    ])
