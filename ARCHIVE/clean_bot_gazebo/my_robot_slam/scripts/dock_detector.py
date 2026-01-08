#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
import math


class DockDetector(Node):
    def __init__(self):
        super().__init__('dock_detector')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.publisher = self.create_publisher(PoseStamped, '/my_dock_pose', 10)
        self.get_logger().info('Dock Detector Started')

    def scan_callback(self, msg):
        ranges = msg.ranges
        angle_inc = msg.angle_increment
        window = 20
        
        best_score = float('inf')
        best_index = -1
        
        # Iterate through scan to find V-shape candidates
        # We skip the edges
        for i in range(window, len(ranges) - window):
            dist = ranges[i]
            
            # 1. Range Check
            if dist > 5.0: 
                continue
                
            # 2. Local Minimum Check (Simple)
            # Must be smaller than neighbors (or equal)
            if ranges[i-1] < dist or ranges[i+1] < dist:
                continue
                
            # 3. Depth Check
            left_edge = ranges[i - window]
            right_edge = ranges[i + window]
            
            # Must be significantly deeper than the window edges
            if not (left_edge > dist + 0.05 and right_edge > dist + 0.05):
                continue
                
            # 4. Angle Check (Law of Cosines)
            # Center point
            cx = dist # Relative to scanner, center is at distance d along ray 0 (locally)
            cy = 0.0
            
            # Left point
            theta_left = window * angle_inc
            lx = left_edge * math.cos(theta_left)
            ly = left_edge * math.sin(theta_left)
            
            # Right point
            theta_right = -window * angle_inc
            rx = right_edge * math.cos(theta_right)
            ry = right_edge * math.sin(theta_right)
            
            # Vectors from Center to Left/Right
            v1x = lx - cx; v1y = ly - cy
            v2x = rx - cx; v2y = ry - cy
            
            mag1 = math.hypot(v1x, v1y)
            mag2 = math.hypot(v2x, v2y)
            
            if mag1 == 0 or mag2 == 0: continue
            
            dot = v1x * v2x + v1y * v2y
            cos_angle = dot / (mag1 * mag2)
            cos_angle = max(-1.0, min(1.0, cos_angle))
            angle_deg = math.degrees(math.acos(cos_angle))
            
            # Filter: Dock should be sharp (< 85 degrees)
            if angle_deg < 85:
                # self.get_logger().info(f"Candidate: dist={dist:.2f}, angle={angle_deg:.2f}, index={i}")
                # Found a candidate. Prefer the closest one.
                if dist < best_score:
                    best_score = dist
                    best_index = i

        if best_index == -1:
            return

        # Found the dock!
        min_dist = ranges[best_index]
        min_index = best_index
        
        # Calculate Pose of the Dock relative to Robot
        dock_angle = msg.angle_min + min_index * msg.angle_increment
        
        # Publish Dock Pose
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose.position.x = min_dist * math.cos(dock_angle)
        pose.pose.position.y = min_dist * math.sin(dock_angle)
        
        # Orientation: The dock faces the robot, so orientation is roughly opposite to the ray
        # But for the docker, we just need the position.
        # Let's set orientation to point AT the dock
        pose.pose.orientation.w = 1.0 
        
        self.get_logger().info(f"Dock found! Publishing pose. Dist: {min_dist:.2f}, Angle: {dock_angle:.2f}")
        self.publisher.publish(pose)
        self.get_logger().info("Published!")

def main(args=None):
    rclpy.init(args=args)
    node = DockDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
