from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_slam')

    # Simulation Node (Replaces Hardware)
    sim_node = Node(
        package='my_robot_slam',
        executable='simple_simulation.py',
        name='simple_simulation',
        parameters=[{'x_start': 1.5, 'y_start': 1.5, 'theta_start': -1.57}]
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
        name='auto_docker'
    )

    # Robot State Publisher
    urdf_file = os.path.join(pkg_share, 'urdf', 'my_robot.urdf')
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': open(urdf_file).read()}]
    )

    # RViz (use your rviz config file)
    rviz_config = os.path.join(pkg_share, 'config', 'docking_demo.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        sim_node,
        rsp_node,
        detector_node,
        docker_node,
        rviz_node
    ])
