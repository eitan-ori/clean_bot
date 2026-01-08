import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'my_robot_slam'
    pkg_dir = get_package_share_directory(pkg_name)
    
    # Paths
    urdf_file = os.path.join(pkg_dir, 'urdf', 'my_robot.urdf')
    slam_params = os.path.join(pkg_dir, 'config', 'mapper_params_online_async.yaml')
    nav2_params = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')
    
    # 1. Simulation (Complex Map)
    simulation = Node(
        package=pkg_name,
        executable='simple_simulation.py',
        name='simple_simulation',
        parameters=[{
            'map_name': 'complex',
            'x_start': 0.0,
            'y_start': 0.0,
            'theta_start': 0.0
        }]
    )
    
    # 2. Robot State Publisher
    with open(urdf_file, 'r') as inf:
        robot_desc = inf.read()
        
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )
    
    # 3. SLAM Toolbox
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
        launch_arguments={'params_file': slam_params, 'use_sim_time': 'true'}.items()
    )
    
    # 4. Nav2 (Navigation only, no map_server/amcl)
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')),
        launch_arguments={'params_file': nav2_params, 'use_sim_time': 'true'}.items()
    )
    
    # 5. Docking Stack
    dock_detector = Node(package=pkg_name, executable='dock_detector.py', name='dock_detector')
    auto_docker = Node(package=pkg_name, executable='auto_docker.py', name='auto_docker')
    
    # 6. Explorer (Delayed to ensure Nav2 is up)
    explorer = Node(
        package=pkg_name, 
        executable='autonomous_explorer.py', 
        name='autonomous_explorer',
        output='screen'
    )
    
    # Delay explorer start
    delayed_explorer = TimerAction(period=20.0, actions=[explorer])

    return LaunchDescription([
        simulation,
        rsp,
        slam,
        nav2,
        dock_detector,
        auto_docker,
        delayed_explorer
    ])
