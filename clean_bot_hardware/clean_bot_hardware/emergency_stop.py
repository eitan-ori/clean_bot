#!/usr/bin/env python3
"""
Emergency Stop Controller

Monitors ultrasonic sensor for immediate collision threats and stops the robot.
This is a safety layer that overrides navigation commands when obstacle is too close.

This node acts as a "velocity mux" - it passes through cmd_vel normally,
but blocks/modifies it when an obstacle is detected at close range.

Sensor configuration:
- Ultrasonic at 3cm height: detects low obstacles
- Emergency stop distance: 10cm (stops immediately)
- Slow down distance: 30cm (reduces speed)

Topics Subscribed:
- /ultrasonic_range (sensor_msgs/Range) - Ultrasonic readings
- /cmd_vel_nav (geometry_msgs/Twist) - Input from Nav2

Topics Published:
- /cmd_vel (geometry_msgs/Twist) - Output to motors (filtered)

Author: Clean Bot Team
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from std_msgs.msg import Bool


class EmergencyStopController(Node):
    def __init__(self):
        super().__init__('emergency_stop_controller')

        # ===================== Parameters =====================
        self.declare_parameter('emergency_stop_distance', 0.10)  # 10cm - full stop
        self.declare_parameter('slow_down_distance', 0.30)       # 30cm - reduce speed
        self.declare_parameter('slow_down_factor', 0.3)          # 30% of original speed
        self.declare_parameter('reverse_allowed', True)          # Allow backing up
        self.declare_parameter('timeout_sec', 0.5)               # Sensor timeout
        
        self.stop_dist = self.get_parameter('emergency_stop_distance').value
        self.slow_dist = self.get_parameter('slow_down_distance').value
        self.slow_factor = self.get_parameter('slow_down_factor').value
        self.reverse_allowed = self.get_parameter('reverse_allowed').value
        self.timeout = self.get_parameter('timeout_sec').value

        # ===================== Publishers =====================
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.obstacle_pub = self.create_publisher(Bool, 'obstacle_detected', 10)

        # ===================== Subscribers =====================
        self.range_sub = self.create_subscription(
            Range, 'ultrasonic_range', self.range_callback, 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel_nav', self.cmd_vel_callback, 10)

        # ===================== State =====================
        self.current_distance = float('inf')
        self.last_range_time = None
        self.obstacle_state = False  # False = clear, True = blocked

        self.get_logger().info('🛑 Emergency Stop Controller started')
        self.get_logger().info(f'   Stop distance: {self.stop_dist*100:.0f}cm')
        self.get_logger().info(f'   Slow distance: {self.slow_dist*100:.0f}cm')

    def range_callback(self, msg: Range):
        """Update current obstacle distance."""
        self.current_distance = msg.range
        self.last_range_time = self.get_clock().now()

    def cmd_vel_callback(self, msg: Twist):
        """
        Filter velocity commands based on obstacle proximity.
        Passes through commands but limits forward motion when obstacle detected.
        """
        output = Twist()
        output.angular.z = msg.angular.z  # Always allow rotation
        
        # Check if sensor data is fresh
        if self.last_range_time is not None:
            elapsed = (self.get_clock().now() - self.last_range_time).nanoseconds / 1e9
            if elapsed > self.timeout:
                # Sensor timeout - be cautious, reduce forward speed
                self.get_logger().warn_once('⚠️ Ultrasonic sensor timeout - reducing speed')
                output.linear.x = msg.linear.x * 0.5 if msg.linear.x > 0 else msg.linear.x
                self.cmd_vel_pub.publish(output)
                return

        # Check distance and apply safety limits
        distance = self.current_distance
        
        if distance <= self.stop_dist:
            # EMERGENCY STOP - obstacle too close!
            if msg.linear.x > 0:  # Only stop forward motion
                if not self.obstacle_state:
                    self.get_logger().warn(f'🛑 EMERGENCY STOP! Obstacle at {distance*100:.0f}cm')
                    self.obstacle_state = True
                    self.publish_obstacle_state(True)
                
                output.linear.x = 0.0
                
                # Allow backing up if enabled
                if self.reverse_allowed:
                    output.linear.x = -0.05  # Slow reverse
            else:
                output.linear.x = msg.linear.x  # Allow reverse
                
        elif distance <= self.slow_dist:
            # SLOW DOWN ZONE
            if msg.linear.x > 0:
                # Scale speed based on distance
                # At slow_dist: full speed, at stop_dist: slow_factor
                scale = self.slow_factor + (1 - self.slow_factor) * \
                        (distance - self.stop_dist) / (self.slow_dist - self.stop_dist)
                output.linear.x = msg.linear.x * scale
                
                self.get_logger().debug(
                    f'⚠️ Slowing down ({distance*100:.0f}cm) - speed at {scale*100:.0f}%')
            else:
                output.linear.x = msg.linear.x
                
            if self.obstacle_state:
                self.obstacle_state = False
                self.publish_obstacle_state(False)
        else:
            # CLEAR - pass through normally
            output.linear.x = msg.linear.x
            
            if self.obstacle_state:
                self.get_logger().info('✅ Obstacle cleared')
                self.obstacle_state = False
                self.publish_obstacle_state(False)
        
        self.cmd_vel_pub.publish(output)

    def publish_obstacle_state(self, blocked: bool):
        """Publish obstacle detection state."""
        msg = Bool()
        msg.data = blocked
        self.obstacle_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = EmergencyStopController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
