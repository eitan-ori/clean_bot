#!/usr/bin/env python3
"""
###############################################################################
# FILE DESCRIPTION:
# This node implements a safety supervisor (Emergency Stop) for the robot.
# It monitors the distance to obstacles detected by the ultrasonic sensor and
# modifies incoming velocity commands from navigation to prevent collisions.
#
# MAIN FUNCTIONS:
# 1. Subscribes to /cmd_vel_nav (input from the navigation stack).
# 2. Subscribes to /ultrasonic_range (real-time distance to obstacles).
# 3. Filters forward velocity:
#    - Obstacle < distance_stop: Linear velocity set to 0 (Full Stop).
#    - Obstacle < distance_slow: Linear velocity scaled down (Caution).
#    - Obstacle > distance_slow: Velocity passes through unchanged.
# 4. Publishes final vetted commands to /cmd_vel.
#
# PARAMETERS & VALUES:
# - emergency_stop_distance: 0.10 m (Triggers immediate halt).
# - slow_down_distance: 0.30 m (Triggers speed reduction).
# - slow_down_factor: 0.3 (Multiply original speed by 0.3 in caution zone).
# - reverse_allowed: True (Allows backing away from obstacles).
# - timeout_sec: 0.5 s (Safety fallback if sensor data stops arriving).
#
# ASSUMPTIONS:
# - The navigation stack outputs to /cmd_vel_nav instead of /cmd_vel.
# - The ultrasonic sensor is facing forward and providing accurate range data.
# - Rotation is always allowed even in stop state (to let the robot turn away).
###############################################################################
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
        self.current_distance = 100
        self.last_range_time = None
        self.obstacle_state = False  # False = clear, True = blocked

        self.get_logger().info('🛑 Emergency Stop Controller started')
        self.get_logger().info(f'   Stop distance: {self.stop_dist*100:.0f}cm')
        self.get_logger().info(f'   Slow distance: {self.slow_dist*100:.0f}cm')

    def range_callback(self, msg: Range):
        """Update current obstacle distance."""
        # Ignore 0 readings - HC-SR04 returns 0 when no echo received
        # Also ignore readings below min_range (typically 2cm for HC-SR04)
        if msg.range <= 0.02:  # 2cm minimum valid reading
            # Invalid reading - don't update distance (keep previous valid value)
            return
        
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
