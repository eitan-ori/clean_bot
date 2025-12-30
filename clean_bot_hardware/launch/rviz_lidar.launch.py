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
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        ),
    ])
