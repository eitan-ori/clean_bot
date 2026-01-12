#!/usr/bin/env python3
"""
###############################################################################
# FILE DESCRIPTION:
# This node converts 1D distance data from an ultrasonic sensor into a 3D
# PointCloud2 message. This allows the navigation stack to "see" low-profile
# obstacles that would normally be invisible to a planar LiDAR mounted higher
# on the robot.
#
# MAIN FUNCTIONS:
# 1. Subscribes to /ultrasonic_range.
# 2. When an obstacle is within range, it generates a set of virtual points
#    in a 3D arc representing the sensor's field of view.
# 3. Publishes a PointCloud2 to /low_obstacles for integration into Nav2 costmaps.
# 4. Publishes a visualization marker to /low_obstacle_marker for RViz.
#
# PARAMETERS & VALUES:
# - ultrasonic_frame: ultrasonic_link (TF frame associated with the sensor).
# - min_obstacle_distance: 0.05 m (Ignore readings closer than 5cm due to noise).
# - max_obstacle_distance: 0.50 m (Only report obstacles closer than 50cm).
# - obstacle_height: 0.03 m (Assumed height of the "virtual" points).
# - cone_angle: 0.26 rad (~15 deg half-angle of the sensor beam).
# - points_per_detection: 5 (Number of points used to represent one detection).
# - obstacle_persistence: 2.0 s (How long to keep publishing the obstacle after detection).
#
# ASSUMPTIONS:
# - The ultrasonic sensor is mounted at a low height (e.g., 3cm).
# - Obstacles detected by the ultrasonic sensor are standing on the floor.
# - Nav2 is configured to include the /low_obstacles topic in its observation sources.
###############################################################################
"""

import math
import struct
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from sensor_msgs.msg import Range, PointCloud2, PointField
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Header


class LowObstacleDetector(Node):
    def __init__(self):
        super().__init__('low_obstacle_detector')

        # ===================== Parameters =====================
        self.declare_parameter('ultrasonic_frame', 'ultrasonic_link')
        self.declare_parameter('min_obstacle_distance', 0.05)   # 5cm - ignore closer (noise)
        self.declare_parameter('max_obstacle_distance', 0.50)   # 50cm - detection range
        self.declare_parameter('obstacle_height', 0.03)         # 3cm - sensor height
        self.declare_parameter('cone_angle', 0.26)              # ~15 degrees half-angle
        self.declare_parameter('points_per_detection', 5)       # Points to generate per reading
        self.declare_parameter('obstacle_persistence', 2.0)     # Seconds to keep obstacle
        
        self.frame_id = self.get_parameter('ultrasonic_frame').value
        self.min_dist = self.get_parameter('min_obstacle_distance').value
        self.max_dist = self.get_parameter('max_obstacle_distance').value
        self.obstacle_height = self.get_parameter('obstacle_height').value
        self.cone_angle = self.get_parameter('cone_angle').value
        self.num_points = self.get_parameter('points_per_detection').value
        self.persistence = self.get_parameter('obstacle_persistence').value

        # ===================== Publishers =====================
        self.pointcloud_pub = self.create_publisher(
            PointCloud2, 'low_obstacles', 10)
        self.marker_pub = self.create_publisher(
            Marker, 'low_obstacle_marker', 10)

        # ===================== Subscribers =====================
        self.range_sub = self.create_subscription(
            Range, 'ultrasonic_range', self.range_callback, 10)

        # ===================== State =====================
        self.last_obstacle_time = None
        self.last_obstacle_distance = None
        
        # Publish rate for persistent obstacles
        self.create_timer(0.1, self.publish_persistent_obstacle)

        self.get_logger().info('🔍 Low Obstacle Detector started')
        self.get_logger().info(f'   Detection range: {self.min_dist*100:.0f}cm - {self.max_dist*100:.0f}cm')
        self.get_logger().info(f'   Sensor height: {self.obstacle_height*100:.0f}cm')

    def range_callback(self, msg: Range):
        """Process ultrasonic range reading."""
        distance = msg.range
        
        # Validate reading
        if distance < self.min_dist or distance > self.max_dist:
            return
        
        if distance >= msg.max_range:
            # No obstacle detected - clear
            self.last_obstacle_distance = None
            return
        
        # Obstacle detected!
        self.last_obstacle_time = self.get_clock().now()
        self.last_obstacle_distance = distance
        
        self.get_logger().debug(f'⚠️ Low obstacle at {distance*100:.1f}cm')
        
        # Generate and publish point cloud
        self.publish_obstacle_pointcloud(distance)
        self.publish_obstacle_marker(distance)

    def publish_persistent_obstacle(self):
        """Re-publish obstacle if still within persistence window."""
        if self.last_obstacle_distance is None or self.last_obstacle_time is None:
            return
        
        elapsed = (self.get_clock().now() - self.last_obstacle_time).nanoseconds / 1e9
        if elapsed < self.persistence:
            self.publish_obstacle_pointcloud(self.last_obstacle_distance)
        else:
            # Obstacle expired
            self.last_obstacle_distance = None

    def publish_obstacle_pointcloud(self, distance: float):
        """
        Generate PointCloud2 representing the detected obstacle.
        Creates a small arc of points at the obstacle location.
        """
        points = []
        
        # Generate points in a cone pattern
        for i in range(self.num_points):
            # Spread points across the cone angle
            angle = -self.cone_angle + (2 * self.cone_angle * i / (self.num_points - 1)) if self.num_points > 1 else 0
            
            # Point position relative to sensor
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)
            z = 0.0  # At sensor height (will be transformed by TF)
            
            points.append((x, y, z))
        
        # Create PointCloud2 message
        header = Header()
        # Use Time(0) to tell the costmap to use latest available transform
        # This avoids timing issues where message timestamp is ahead of TF buffer
        header.stamp = Time(seconds=0).to_msg()
        header.frame_id = self.frame_id
        
        # Define fields
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        # Pack point data
        cloud_data = []
        for p in points:
            cloud_data.append(struct.pack('fff', p[0], p[1], p[2]))
        
        # Create message
        pc2 = PointCloud2()
        pc2.header = header
        pc2.height = 1
        pc2.width = len(points)
        pc2.fields = fields
        pc2.is_bigendian = False
        pc2.point_step = 12  # 3 floats * 4 bytes
        pc2.row_step = pc2.point_step * len(points)
        pc2.data = b''.join(cloud_data)
        pc2.is_dense = True
        
        self.pointcloud_pub.publish(pc2)

    def publish_obstacle_marker(self, distance: float):
        """Publish visualization marker for RViz."""
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'low_obstacles'
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        
        # Position at detected distance
        marker.pose.position.x = distance
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # Size - small cylinder representing obstacle
        marker.scale.x = 0.10  # Diameter
        marker.scale.y = 0.10
        marker.scale.z = 0.05  # Height
        
        # Color - orange warning
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
        marker.color.a = 0.8
        
        # Lifetime
        marker.lifetime.sec = int(self.persistence)
        marker.lifetime.nanosec = int((self.persistence % 1) * 1e9)
        
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = LowObstacleDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
