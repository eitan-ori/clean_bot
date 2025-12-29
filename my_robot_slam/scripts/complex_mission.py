#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import math

class ComplexMission(Node):
    def __init__(self):
        super().__init__('complex_mission')
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_dock_trigger = self.create_publisher(Bool, '/start_docking', 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.state = 'INIT'
        
        # Robot State
        self.x = 0.0
        self.y = 0.0
        self.yaw = 1.57 # Start facing North approx
        self.odom_received = False
        
        self.get_logger().info("Complex Mission: Closed-Loop Navigation Started")

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        
        # Quaternion to Yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        self.odom_received = True

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def control_loop(self):
        if not self.odom_received:
            return

        cmd = Twist()
        V_MOVE = 0.4
        W_TURN = 0.5
        
        # State Machine based on POSITION, not Time
        
        if self.state == 'INIT':
            # Wait for system to settle
            self.state = 'MOVE_NORTH_CORR_1'
            
        elif self.state == 'MOVE_NORTH_CORR_1':
            # Drive North until y > 2.0 (Center of Room 1 Opening)
            cmd.linear.x = V_MOVE
            # Simple P-controller to keep heading North (1.57)
            err = self.normalize_angle(1.57 - self.yaw)
            cmd.angular.z = err * 1.0
            
            if self.y > 2.0:
                self.state = 'TURN_WEST_TO_ROOM_1'

        elif self.state == 'TURN_WEST_TO_ROOM_1':
            # Turn Left to Face West (3.14)
            target = 3.14
            err = self.normalize_angle(target - self.yaw)
            cmd.angular.z = 0.5 if err > 0 else -0.5
            
            if abs(err) < 0.1:
                self.state = 'ENTER_ROOM_1'

        elif self.state == 'ENTER_ROOM_1':
            # Drive West into Room 1 until x < -1.5
            cmd.linear.x = V_MOVE
            err = self.normalize_angle(3.14 - self.yaw)
            cmd.angular.z = err * 1.0
            
            if self.x < -1.5:
                self.state = 'SWEEP_ROOM_1'
                self.sweep_start_time = self.get_clock().now()

        elif self.state == 'SWEEP_ROOM_1':
            # Time-based sweep is okay here as we are safely inside
            # Do a 360 scan
            cmd.angular.z = 0.6
            elapsed = (self.get_clock().now() - self.sweep_start_time).nanoseconds / 1e9
            if elapsed > 10.0: # Full circle + overlap
                self.state = 'EXIT_ROOM_1'

        elif self.state == 'EXIT_ROOM_1':
            # Drive East back to Corridor (x > -0.5)
            # We are likely facing random direction, so turn East first
            target = 0.0
            err = self.normalize_angle(target - self.yaw)
            
            if abs(err) > 0.2:
                cmd.angular.z = 0.5 if err > 0 else -0.5
            else:
                cmd.linear.x = V_MOVE
                cmd.angular.z = err * 1.0
                if self.x > -0.1: # Ensure we are back in center of corridor (x=0)
                    self.state = 'MOVE_TO_TOP_CORR'

        elif self.state == 'MOVE_TO_TOP_CORR':
            # Drive North to y > 3.5
            target = 1.57
            err = self.normalize_angle(target - self.yaw)
            
            if abs(err) > 0.2:
                cmd.angular.z = 0.5 if err > 0 else -0.5
            else:
                cmd.linear.x = V_MOVE
                cmd.angular.z = err * 1.0
                if self.y > 3.5:
                    self.state = 'TURN_EAST_CORR_2'

        elif self.state == 'TURN_EAST_CORR_2':
            # Turn Right to East (0.0)
            target = 0.0
            err = self.normalize_angle(target - self.yaw)
            cmd.angular.z = -0.5 # Force right turn
            
            if abs(err) < 0.1:
                self.state = 'TRAVERSE_CORR_2'

        elif self.state == 'TRAVERSE_CORR_2':
            # Drive East until x > 3.0
            cmd.linear.x = V_MOVE
            err = self.normalize_angle(0.0 - self.yaw)
            cmd.angular.z = err * 1.0
            
            if self.x > 3.0:
                self.state = 'TURN_SOUTH_ROOM_2'

        elif self.state == 'TURN_SOUTH_ROOM_2':
            # Turn Right to South (-1.57)
            target = -1.57
            err = self.normalize_angle(target - self.yaw)
            cmd.angular.z = -0.5
            
            if abs(err) < 0.1:
                self.state = 'ENTER_ROOM_2'

        elif self.state == 'ENTER_ROOM_2':
            # Drive South into Room 2 until y < 2.0
            cmd.linear.x = V_MOVE
            err = self.normalize_angle(-1.57 - self.yaw)
            cmd.angular.z = err * 1.0
            
            if self.y < 2.0:
                self.state = 'PRE_DOCK'

        elif self.state == 'PRE_DOCK':
            # Stop and trigger docking
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            
            msg = Bool()
            msg.data = True
            self.pub_dock_trigger.publish(msg)
            self.state = 'DOCKING'
            self.get_logger().info("Handing over to AutoDocker")

        self.pub_cmd.publish(cmd)
        
        # Debug log every 1s
        # self.get_logger().info(f"State: {self.state} | Pose: ({self.x:.2f}, {self.y:.2f}, {self.yaw:.2f})")

def main(args=None):
    rclpy.init(args=args)
    node = ComplexMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

