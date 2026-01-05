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
                'laser_scan_topic': '/scan',     # וודא שזה הטופיק של הלידאר שלך
                'odom_topic': '/odom',           # הטופיק שייווצר
                'publish_tf': True,              # חשוב! זה מה שיוצר את החיבור החסר
                'base_frame_id': 'base_link',    # גוף הרובוט
                'odom_frame_id': 'odom',         # מערכת הצירים העולמית
                'init_pose_from_topic': '',
                'freq': 20.0                     # קצב עדכון
            }],
        ),
    ])