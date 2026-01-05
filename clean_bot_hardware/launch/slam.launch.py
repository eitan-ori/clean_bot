import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Paths to packages
    hardware_pkg = get_package_share_directory('clean_bot_hardware')
    description_pkg = get_package_share_directory('clean_bot_description')
    slam_pkg = get_package_share_directory('slam_toolbox')

    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # 1. Launch Sensors (Lidar + IMU)
    # Disable IMU odom because we use rf2o
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hardware_pkg, 'launch', 'sensors.launch.py')
        ),
        launch_arguments={'publish_odom': 'false'}.items()
    )

    # 2. Launch Robot State Publisher
    # rsp_launch is defined above...

    # 2.1 Joint State Publisher (Fixes RViz wheel errors)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
    )

    # 3. Launch SLAM Toolbox
    slam_params_file = os.path.join(hardware_pkg, 'config', 'mapper_params_online_async.yaml')
    
    start_slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_pkg, 'launch', 'lifelong_launch.py')
        ),
        launch_arguments={
            'params_file': slam_params_file,
            'use_sim_time': use_sim_time
        }.items()
    )

    # 4. RF2O Laser Odometry
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom',
            'publish_tf': True,
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 10.0
        }],
    )

    # 5. Launch RViz (Disabled for headless Pi)
    # rviz_config = os.path.join(hardware_pkg, 'config', 'rplidar_rviz.rviz')
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', rviz_config],
    #     output='screen'
    # )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation/Gazebo clock'),
        
        sensors_launch,
        # rsp_launch,
        joint_state_publisher_node,
        rf2o_node,
        start_slam_toolbox_launch,
        # rviz_node
    ])
