"""Launch RPLidar A1M8 driver and test node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for RPLidar test."""
    
    # Declare arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for RPLidar'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='laser_frame',
        description='Frame ID for laser scans'
    )
    
    # RPLidar driver node
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'serial_baudrate': 115200,  # A1M8 uses 115200
            'frame_id': LaunchConfiguration('frame_id'),
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard',  # A1 uses Standard mode
        }],
        output='screen'
    )
    
    # Test node (delayed start to allow driver to initialize)
    test_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='clean_bot_hardware',
                executable='rplidar_test',
                name='rplidar_test',
                parameters=[{
                    'scan_topic': '/scan',
                    'display_interval': 0.5,
                }],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        serial_port_arg,
        frame_id_arg,
        rplidar_node,
        test_node,
    ])
