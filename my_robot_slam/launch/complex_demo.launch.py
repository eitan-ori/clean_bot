from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_slam')

    sim_speed_arg = DeclareLaunchArgument(
        'sim_speed',
        default_value='1.0',
        description='Simulation speed multiplier (e.g. 2.0 = 2x faster)'
    )

    # Simulation Node (Complex Map)
    sim_node = Node(
        package='my_robot_slam',
        executable='simple_simulation.py',
        name='simple_simulation',
        remappings=[('/cmd_vel', '/cmd_vel_muxed')],
        parameters=[{
            'x_start': 0.0, 
            'y_start': 0.0, 
            'theta_start': 1.57, # Face North
            'map_name': 'complex',
            'sim_speed': LaunchConfiguration('sim_speed'),
        }]
    )

    # Dock Detector
    detector_node = Node(
        package='my_robot_slam',
        executable='dock_detector.py',
        name='dock_detector'
    )

    # Auto Docker
    docker_node = Node(
        package='my_robot_slam',
        executable='auto_docker.py',
        name='auto_docker',
        remappings=[('/cmd_vel', '/cmd_vel_dock')],
    )

    # cmd_vel Mux: ensures exactly one command stream reaches the simulator
    mux_node = Node(
        package='my_robot_slam',
        executable='cmd_vel_mux.py',
        name='cmd_vel_mux',
        output='screen',
        parameters=[{
            'publish_rate_hz': 20.0,
            'input_timeout_sec': 0.5,
            'dock_latch_sec': 60.0,
        }],
    )

    # Robot State Publisher
    urdf_file = os.path.join(pkg_share, 'urdf', 'my_robot.urdf')
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': open(urdf_file).read()}]
    )

    # SLAM Toolbox (Async)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'online_async_launch.py')
        )
    )

    # Autonomous Explorer (Replaces Mission Controller)
    explorer_node = Node(
        package='my_robot_slam',
        executable='autonomous_explorer.py',
        name='autonomous_explorer',
        output='screen',
        remappings=[('/cmd_vel', '/cmd_vel_explore')],
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
        sim_speed_arg,
        sim_node,
        rsp_node,
        detector_node,
        docker_node,
        mux_node,
        slam_launch,
        explorer_node,
        rviz_node
    ])
