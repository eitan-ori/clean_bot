#!/usr/bin/env python3
"""
###############################################################################
# FILE DESCRIPTION:
# This node provides a simplified transformation broadcaster that maps IMU
# orientation directly to the robot's odometry frame.
#
# MAIN FUNCTIONS:
# 1. Subscribes to IMU data (orientation quaternion).
# 2. Publishes a TF transform from 'odom' to 'base_link'.
# 3. Keeps the robot's translation at zero while updating its rotation.
#
# PARAMETERS & VALUES:
# - imu_topic: /imu/data (Source of absolute orientation).
# - parent_frame: odom (The reference world/start frame).
# - child_frame: base_link (The robot's local frame).
#
# ASSUMPTIONS:
# - This node is intended for orientation-only testing or systems where 
#   linear displacement is handled by a different source (or not needed).
# - The IMU sensor is mounted level and aligned with the robot's forward axis.
# - The IMU provides a filtered orientation estimate (not raw gyro data).
###############################################################################
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class ImuOdomBroadcaster(Node):
    def __init__(self):
        super().__init__('imu_odom_broadcaster')
        
        # Parameters
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('child_frame', 'base_link')
        
        imu_topic = self.get_parameter('imu_topic').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value
        
        # Subscriber
        self.subscription = self.create_subscription(
            Imu,
            imu_topic,
            self.imu_callback,
            10)
            
        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info(f"Broadcasting {self.parent_frame} -> {self.child_frame} based on {imu_topic}")

    def imu_callback(self, msg):
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame

        # Use IMU orientation for the rotation
        # We assume the robot stays at (0,0,0) since we only have IMU
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        t.transform.rotation = msg.orientation

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = ImuOdomBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
