#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import time

class DemoMission(Node):
    def __init__(self):
        super().__init__('demo_mission')
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_dock_trigger = self.create_publisher(Bool, '/start_docking', 10)
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.start_time = self.get_clock().now()
        self.state = 'INIT'
        
        self.get_logger().info("Demo Mission Started")

    def control_loop(self):
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        
        cmd = Twist()
        
        # Sequence for L-Shaped Room Coverage (Vacuum Pattern)
        # Start at (-1.0, -1.0) facing East (0.0)
        
        # Speeds
        LINEAR_SPEED = 0.5
        ANGULAR_SPEED = 0.5 
        
        # Revised Path with Safer Margins (0.8m from walls)
        # 1. Init
        # 2. Spin L/R
        # 3. Move East 2.2m -> (1.2, -1.0)
        # 4. Turn North
        # 5. Move North 2.2m -> (1.2, 1.2)
        # 6. Turn West
        # 7. Move West 0.4m -> (0.8, 1.2)
        # 8. Turn South
        # 9. Move South 2.0m -> (0.8, -0.8)
        # 10. Turn West (Right)
        # 11. Move West 2.0m -> (-1.2, -0.8)
        # 12. Turn South (Left)
        # 13. Move South 0.4m -> (-1.2, -1.2)
        # 14. Turn East (Left)
        # 15. Move East 2.2m -> (1.0, -1.2)
        # 16. Turn North (Left)
        # 17. Move North 1.2m -> (1.0, 0.0)
        # 18. Turn East (Right) -> Face Dock
        
        if elapsed < 1.0:
            self.state = "INIT"
            
        elif elapsed < 4.2:
            self.state = "MAPPING_SPIN_L"
            cmd.angular.z = ANGULAR_SPEED
            
        elif elapsed < 7.4:
            self.state = "MAPPING_SPIN_R"
            cmd.angular.z = -ANGULAR_SPEED
            
        elif elapsed < 11.8:
            self.state = "SWEEP_BOTTOM_1"
            cmd.linear.x = LINEAR_SPEED
            
        elif elapsed < 15.0:
            self.state = "TURN_NORTH_1"
            cmd.angular.z = ANGULAR_SPEED
            
        elif elapsed < 19.4:
            self.state = "SWEEP_RIGHT_EDGE"
            cmd.linear.x = LINEAR_SPEED
            
        elif elapsed < 22.6:
            self.state = "TURN_WEST_1"
            cmd.angular.z = ANGULAR_SPEED
            
        elif elapsed < 23.4:
            self.state = "SWEEP_TOP"
            cmd.linear.x = LINEAR_SPEED
            
        elif elapsed < 26.6:
            self.state = "TURN_SOUTH_1"
            cmd.angular.z = ANGULAR_SPEED
            
        elif elapsed < 30.6:
            self.state = "SWEEP_MIDDLE"
            cmd.linear.x = LINEAR_SPEED
            
        elif elapsed < 33.8:
            self.state = "TURN_WEST_2"
            cmd.angular.z = -ANGULAR_SPEED # Right Turn
            
        elif elapsed < 37.8:
            self.state = "SWEEP_LEFT_TOP"
            cmd.linear.x = LINEAR_SPEED
            
        elif elapsed < 41.0:
            self.state = "TURN_SOUTH_2"
            cmd.angular.z = ANGULAR_SPEED
            
        elif elapsed < 41.8:
            self.state = "SWEEP_LEFT_EDGE"
            cmd.linear.x = LINEAR_SPEED
            
        elif elapsed < 45.0:
            self.state = "TURN_EAST_1"
            cmd.angular.z = ANGULAR_SPEED
            
        elif elapsed < 49.4:
            self.state = "SWEEP_BOTTOM_2"
            cmd.linear.x = LINEAR_SPEED
            
        elif elapsed < 52.6:
            self.state = "TURN_NORTH_2"
            cmd.angular.z = ANGULAR_SPEED
            
        elif elapsed < 55.0:
            self.state = "APPROACH_DOCK_POS"
            cmd.linear.x = LINEAR_SPEED
            
        elif elapsed < 58.2:
            self.state = "FACE_DOCK"
            cmd.angular.z = -ANGULAR_SPEED # Right Turn
            
        elif elapsed < 59.2:
             self.state = "WAIT"
             cmd.linear.x = 0.0
             cmd.angular.z = 0.0

        elif elapsed < 60.2:
            self.state = "TRIGGER_DOCKING"
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            
            trigger = Bool()
            trigger.data = True
            self.pub_dock_trigger.publish(trigger)
            self.get_logger().info("Triggered Auto Docking")
            
            # Stop this node from publishing cmd_vel so AutoDocker can take over
            self.destroy_timer(self.timer)
            return

        self.pub_cmd.publish(cmd)
        # self.get_logger().info(f"State: {self.state}, Time: {elapsed:.1f}")

def main(args=None):
    rclpy.init(args=args)
    node = DemoMission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
