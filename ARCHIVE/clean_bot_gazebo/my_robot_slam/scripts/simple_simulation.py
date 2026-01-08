#!/usr/bin/env python3
"""
Simple 2D Simulator for Differential Drive Robot.

Simulates:
- Kinematics (cmd_vel -> odom)
- LaserScan (Raycasting against a virtual map)
- TF (odom -> base_link)
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion, PoseStamped

from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformBroadcaster


class SimpleSimulation(Node):
    def __init__(self):
        super().__init__('simple_simulation')

        # Parameters
        self.declare_parameter('x_start', -1.0)
        self.declare_parameter('y_start', -1.0)
        self.declare_parameter('theta_start', 0.0)
        self.declare_parameter('map_name', 'l_shape') # 'l_shape' or 'complex'
        self.declare_parameter('sim_speed', 1.0)  # Multiply physics integration speed (e.g. 2.0 = 2x faster)

        self.x = self.get_parameter('x_start').value
        self.y = self.get_parameter('y_start').value
        self.theta = self.get_parameter('theta_start').value
        map_name = self.get_parameter('map_name').value
        self.sim_speed = float(self.get_parameter('sim_speed').value)

        # Robot Specs
        self.max_range = 12.0  # Increased range for better SLAM in corridors
        self.scan_samples = 720 # Higher resolution
        self.robot_radius = 0.1  # Reduced radius for smoother navigation

        # Map Definitions
        self.map_lines = []
        
        if map_name == 'complex':
            # Complex Map: Corridor + 2 Rooms
            # Start (0,0) -> North Corr -> West Room -> East Corr -> South Room
            self.map_lines = [
                # 1. Start Box / Corr 1 Right
                ((0.5, -0.5), (0.5, 3.0)),
                # 2. Corr 2 Bottom
                ((0.5, 3.0), (2.0, 3.0)),
                # 3. Room 2 Left
                ((2.0, 3.0), (2.0, 1.0)),
                # 4. Room 2 Bottom
                ((2.0, 1.0), (4.0, 1.0)),
                # 5. Room 2 Right / Corr 2 End
                ((4.0, 1.0), (4.0, 4.0)),
                # 6. Corr 2 Top
                ((4.0, 4.0), (-0.5, 4.0)),
                # 7. Corr 1 Left / Room 1 Right
                ((-0.5, 4.0), (-0.5, 3.0)),
                # 8. Room 1 Top
                ((-0.5, 3.0), (-2.5, 3.0)),
                # 9. Room 1 Left
                ((-2.5, 3.0), (-2.5, 1.0)),
                # 10. Room 1 Bottom
                ((-2.5, 1.0), (-0.5, 1.0)),
                # 11. Corr 1 Left Lower
                ((-0.5, 1.0), (-0.5, -0.5)),
                # 12. Start Bottom
                ((-0.5, -0.5), (0.5, -0.5)),

                # Dock terminal integrated into the start wall (so it doesn't create an unreachable pocket).
                # Endpoints are on the right wall (x=0.5); the V apex protrudes into free space.
                ((0.3, 0.0), (0.5, 0.15)),
                ((0.3, 0.0), (0.5, -0.15))
            ]
        else:
            # Default L-Shaped Room
            self.map_lines = [
                # Bottom Edge
                ((-2.0, -2.0), (2.0, -2.0)),
                # Right Edge
                ((2.0, -2.0), (2.0, 2.0)),
                # Top Edge (Partial)
                ((2.0, 2.0), (0.0, 2.0)),
                # Inner Vertical
                ((0.0, 2.0), (0.0, 0.0)),
                # Inner Horizontal
                ((0.0, 0.0), (-2.0, 0.0)),
                # Left Edge
                ((-2.0, 0.0), (-2.0, -2.0)),

                # V-Shape Dock at (1.5, 0) facing West
                # Corner at (1.5, 0). Sides go back to (1.8, 0.1) and (1.8, -0.1)
                # This creates a sharper angle (~37 deg) for the dock detector to distinguish from room corners (90 deg)
                ((1.4, 0.0), (1.5, 0.5)),
                ((1.6, 0.0), (1.5, 0.5))
            ]

        # Publishers/Subscribers
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.pub_scan = self.create_publisher(LaserScan, '/scan', 10)
        self.pub_path = self.create_publisher(Path, '/trajectory', 10)
        self.pub_walls = self.create_publisher(MarkerArray, '/true_walls', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Path History
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'odom'
        
        # Create Wall Markers
        self.wall_markers = MarkerArray()
        for i, (p1, p2) in enumerate(self.map_lines):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "walls"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.05 # Line width
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0 # Green
            marker.color.b = 0.0
            
            # Points
            from geometry_msgs.msg import Point
            pt1 = Point(); pt1.x = float(p1[0]); pt1.y = float(p1[1])
            pt2 = Point(); pt2.x = float(p2[0]); pt2.y = float(p2[1])
            marker.points = [pt1, pt2]
            
            self.wall_markers.markers.append(marker)

        # Loop
        self.last_time = self.get_clock().now()
        self.create_timer(0.05, self.update)  # 20Hz

        self.v = 0.0
        self.omega = 0.0
        self.last_cmd_time = self.get_clock().now()
        self._watchdog_active = False

    def cmd_vel_callback(self, msg):
        self.v = msg.linear.x
        self.omega = msg.angular.z
        self.last_cmd_time = self.get_clock().now()
        self._watchdog_active = False
        self.get_logger().info(
            f"cmd_vel rx: v={self.v:.2f} omega={self.omega:.2f}",
            throttle_duration_sec=1.0,
        )

    def update(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # Speed up / slow down the simulated physics without changing wall-clock ROS timings.
        dt *= max(0.0, self.sim_speed)

        # Watchdog: Stop if no command for 0.5s
        if (current_time - self.last_cmd_time).nanoseconds / 1e9 > 0.5:
            self.v = 0.0
            self.omega = 0.0
            if not self._watchdog_active:
                self._watchdog_active = True
                self.get_logger().warn("cmd_vel watchdog: no cmd for >0.5s; stopping")

        # 1. Update Kinematics
        self.theta += self.omega * dt
        new_x = self.x + self.v * math.cos(self.theta) * dt
        new_y = self.y + self.v * math.sin(self.theta) * dt

        # Collision Detection
        if not self.check_collision(new_x, new_y):
            self.x = new_x
            self.y = new_y
        else:
            # Collision! 
            # Stop linear motion, but allow angular motion (sliding/turning)
            self.v = 0.0
            # We do NOT revert theta, so the robot can still turn in place.

        # Debug Pose
        # self.get_logger().info(f"Pose: x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f}")

        # 2. Publish Odom & TF
        self.publish_odom(current_time)

        # 3. Simulate & Publish Scan
        self.publish_scan(current_time)
        
        # 4. Publish Wall Markers (Low frequency check not needed for sim)
        # Update timestamps to current time so RViz doesn't complain
        for marker in self.wall_markers.markers:
            marker.header.stamp = current_time.to_msg()
        self.pub_walls.publish(self.wall_markers)

    def check_collision(self, x, y):
        # Check distance to all map lines
        for p1, p2 in self.map_lines:
            dist = self.point_line_distance(x, y, p1, p2)
            if dist < self.robot_radius:
                return True
        return False

    def point_line_distance(self, px, py, p1, p2):
        # Distance from point (px,py) to segment (p1,p2)
        x1, y1 = p1
        x2, y2 = p2
        
        dx = x2 - x1
        dy = y2 - y1
        if dx == 0 and dy == 0:
            return math.hypot(px - x1, py - y1)

        t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy)
        t = max(0, min(1, t))
        
        closest_x = x1 + t * dx
        closest_y = y1 + t * dy
        
        return math.hypot(px - closest_x, py - closest_y)

    def publish_odom(self, current_time):
        q = self.euler_to_quaternion(0, 0, self.theta)

        # TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

        # Odom Msg
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.omega
        self.pub_odom.publish(odom)

        # Path Msg
        pose_stamped = PoseStamped()
        pose_stamped.header = odom.header
        pose_stamped.pose = odom.pose.pose
        self.path_msg.header.stamp = current_time.to_msg()
        self.path_msg.poses.append(pose_stamped)
        
        # Limit path size (Increased for full coverage demo)
        if len(self.path_msg.poses) > 10000:
            self.path_msg.poses.pop(0)
            
        self.pub_path.publish(self.path_msg)


    def publish_scan(self, current_time):
        scan = LaserScan()
        scan.header.stamp = current_time.to_msg()
        scan.header.frame_id = 'laser_frame'
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = (2 * math.pi) / self.scan_samples
        scan.time_increment = 0.0
        scan.range_min = 0.1
        scan.range_max = self.max_range

        ranges = []
        for i in range(self.scan_samples):
            angle = scan.angle_min + i * scan.angle_increment
            global_angle = self.theta + angle

            # Raycast
            r = self.raycast(self.x, self.y, global_angle)
            ranges.append(r)

        scan.ranges = ranges
        self.pub_scan.publish(scan)

    def raycast(self, x, y, angle):
        min_dist = self.max_range

        # Ray direction
        dx = math.cos(angle)
        dy = math.sin(angle)

        for p1, p2 in self.map_lines:
            dist = self.intersect(x, y, dx, dy, p1, p2)
            if dist is not None and dist < min_dist:
                min_dist = dist

        return min_dist

    def intersect(self, rx, ry, rdx, rdy, p1, p2):
        # Ray: R(t) = (rx, ry) + t * (rdx, rdy)
        # Segment: S(u) = p1 + u * (p2 - p1)
        # Solve for t, u

        x1, y1 = p1
        x2, y2 = p2

        sdx = x2 - x1
        sdy = y2 - y1

        # Cross product of directions
        det = rdx * sdy - rdy * sdx

        if abs(det) < 1e-6:
            return None  # Parallel

        u = ((rx - x1) * rdy - (ry - y1) * rdx) / det
        t = ((x1 - rx) * sdy - (y1 - ry) * sdx) / -det  # Note: check sign logic

        # Re-derivation for safety:
        # rx + t*rdx = x1 + u*sdx
        # ry + t*rdy = y1 + u*sdy
        # t*rdx - u*sdx = x1 - rx
        # t*rdy - u*sdy = y1 - ry
        # Matrix [rdx  -sdx] [t] = [x1-rx]
        #        [rdy  -sdy] [u]   [y1-ry]
        # Det = -rdx*sdy + sdx*rdy = -(rdx*sdy - rdy*sdx)

        det = rdx * (-sdy) - rdy * (-sdx)
        if abs(det) < 1e-6:
            return None

        dt = (x1 - rx) * (-sdy) - (y1 - ry) * (-sdx)
        du = rdx * (y1 - ry) - rdy * (x1 - rx)

        t = dt / det
        u = du / det

        if t > 0 and 0 <= u <= 1:
            return t
        return None

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - \
            math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + \
            math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - \
            math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + \
            math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleSimulation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
