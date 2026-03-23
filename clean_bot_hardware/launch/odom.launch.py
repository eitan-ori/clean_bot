from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                # IMPORTANT: rf2o must subscribe to the raw scan.
                # /scan is produced by scan_throttle and is intentionally TF-gated.
                # If rf2o subscribes to /scan it creates a circular dependency:
                #   rf2o needs scans to publish odom->base_link TF,
                #   scan_throttle needs that TF to publish /scan.
                'laser_scan_topic': '/scan_raw',
                'odom_topic': '/odom',
                'publish_tf': True,              # Publishes odom→base_link TF
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'init_pose_from_topic': '',
                'freq': 20.0
            }],
        ),
    ])