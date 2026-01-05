#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import numpy as np
import math

def euler_from_quaternion(q):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    x = q.x
    y = q.y
    z = q.z
    w = q.w
    
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z # in radians

class AutonomousExplorer(Node):
    def __init__(self):
        super().__init__('autonomous_explorer')
        
        # Action Client for Nav2
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Publishers
        self.pub_dock_trigger = self.create_publisher(Bool, '/start_docking', 10)
        
        # Subscribers
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # State
        self.map_data = None
        self.map_info = None
        self.robot_pose = None # (x, y, yaw)
        self.start_pose = None # (x, y)
        self.goal_handle = None
        self.is_navigating = False
        self.exploration_active = True
        self.blacklist = []
        self.current_target = None
        
        # Parameters
        self.min_frontier_size = 5
        self.map_resolution = 0.05
        self.map_origin = (0,0)
        
        # Timer for decision making
        self.create_timer(1.0, self.control_loop)
        
        self.get_logger().info("Autonomous Explorer (Nav2) Started")

    def odom_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion(q)
        self.robot_pose = (p.x, p.y, yaw)
        
        if self.start_pose is None:
            self.start_pose = (p.x, p.y)

        # Check if we are returning home and are close enough
        if not self.exploration_active and self.is_navigating:
            dx = p.x - self.start_pose[0]
            dy = p.y - self.start_pose[1]
            dist = math.sqrt(dx*dx + dy*dy)
            
            if dist < 1.5: # 1.5m tolerance
                self.get_logger().info(f"Close to home (dist={dist:.2f}). Cancelling Nav and triggering Docking.")
                self.cancel_navigation()
                self.trigger_docking()

    def trigger_docking(self):
        self.get_logger().info("Triggering Auto Docker...")
        msg = Bool()
        msg.data = True
        self.pub_dock_trigger.publish(msg)
        self.is_navigating = False

    def cancel_navigation(self):
        if self.goal_handle:
            self.get_logger().info("Cancelling current goal...")
            try:
                self.goal_handle.cancel_goal_async()
            except:
                pass
        self.is_navigating = False

    def map_callback(self, msg):
        self.map_info = msg.info
        self.map_resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        
        # Convert to numpy array
        # -1: Unknown, 0: Free, 100: Occupied
        raw = np.array(msg.data, dtype=np.int8)
        self.map_data = raw.reshape((msg.info.height, msg.info.width))

    def control_loop(self):
        if not self.exploration_active:
            return
            
        if self.robot_pose is None or self.map_data is None:
            self.get_logger().info("Waiting for map and odom...")
            return
            
        if self.is_navigating:
            return

        # Find Frontier
        target = self.find_frontier()
        
        if target:
            self.get_logger().info(f"Frontier found at {target}. Navigating...")
            self.send_goal(target)
        else:
            self.get_logger().info("No frontiers found. Exploration Complete.")
            self.return_home()

    def find_frontier(self):
        # Simple frontier detection:
        # Find free cells (0) that are adjacent to unknown cells (-1)
        
        rows, cols = self.map_data.shape
        
        # Optimization: Only check every Nth pixel to save CPU
        step = 2
        candidates = []
        
        for r in range(1, rows-1, step):
            for c in range(1, cols-1, step):
                if self.map_data[r, c] == 0: # If free
                    # Check 4-neighbors for unknown
                    if (self.map_data[r+1, c] == -1 or
                        self.map_data[r-1, c] == -1 or
                        self.map_data[r, c+1] == -1 or
                        self.map_data[r, c-1] == -1):
                        
                        # Convert grid to world
                        wx, wy = self.grid_to_world(c, r)
                        
                        # Check if reachable (simple distance check for now)
                        dist = math.hypot(wx - self.robot_pose[0], wy - self.robot_pose[1])
                        if dist > 0.5: # Don't pick something too close
                            candidates.append((wx, wy))

        if not candidates:
            return None
            
        # Pick the closest candidate that is not blacklisted
        best_candidate = None
        min_dist = float('inf')
        
        for cand in candidates:
            # Check blacklist (simple exact match might fail due to float, use distance)
            is_blacklisted = False
            for b in self.blacklist:
                if math.hypot(cand[0]-b[0], cand[1]-b[1]) < 0.1:
                    is_blacklisted = True
                    break
            if is_blacklisted:
                continue

            dist = math.hypot(cand[0] - self.robot_pose[0], cand[1] - self.robot_pose[1])
            if dist < min_dist:
                min_dist = dist
                best_candidate = cand
                
        return best_candidate

    def grid_to_world(self, gx, gy):
        wx = gx * self.map_resolution + self.map_origin[0]
        wy = gy * self.map_resolution + self.map_origin[1]
        return wx, wy

    def send_goal(self, target):
        self.current_target = target
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = target[0]
        goal_msg.pose.pose.position.y = target[1]
        goal_msg.pose.pose.orientation.w = 1.0 # No specific orientation
        
        self._action_client.wait_for_server()
        self.is_navigating = True
        
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.is_navigating = False
            if self.current_target:
                self.blacklist.append(self.current_target)
            return

        self.get_logger().info('Goal accepted!')
        self.goal_handle = goal_handle
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        self.get_logger().info(f'Navigation finished with status: {status}')
        self.is_navigating = False
        
        # If we were returning home, trigger docking
        if not self.exploration_active:
            self.get_logger().info("Returned to start area. Triggering Auto Docker...")
            self.trigger_docking()

    def return_home(self):
        self.exploration_active = False
        self.get_logger().info("Returning to Start Position...")
        if self.start_pose:
            self.send_goal(self.start_pose)
        else:
            self.send_goal((0.0, 0.0))

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
