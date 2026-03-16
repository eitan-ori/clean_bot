#!/usr/bin/env python3
"""
Scan Diagnostic Monitor
=======================
Lightweight node that traces the velocity command chain during scan mode.
Prints ONLY meaningful events:
  - Map received / map ready
  - Exploration started / frontier selected / goal reached
  - Velocity flow: frontier_explorer → Nav2 → velocity_smoother → emergency_stop → arduino_driver

Subscribes to every topic in the chain and prints which node published what.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import String, Bool
from sensor_msgs.msg import LaserScan


class ScanMonitor(Node):
    def __init__(self):
        super().__init__('scan_monitor')

        self._map_received = False
        self._odom_received = False
        self._scan_received = False
        self._last_cmd_vel_nav = 0.0
        self._last_cmd_vel = 0.0
        self._last_cmd_vel_safe = 0.0
        self._goal_count = 0

        # ── QoS profiles ──
        reliable = QoSProfile(depth=10)
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.get_logger().info('')
        self.get_logger().info('╔══════════════════════════════════════════════════╗')
        self.get_logger().info('║       🔍  SCAN DIAGNOSTIC MONITOR  🔍          ║')
        self.get_logger().info('╠══════════════════════════════════════════════════╣')
        self.get_logger().info('║  Velocity chain being monitored:                ║')
        self.get_logger().info('║                                                 ║')
        self.get_logger().info('║  frontier_explorer                              ║')
        self.get_logger().info('║    → /navigate_to_pose (action)                 ║')
        self.get_logger().info('║  bt_navigator                                   ║')
        self.get_logger().info('║    → controller_server                          ║')
        self.get_logger().info('║      → /cmd_vel_nav  (Twist)                    ║')
        self.get_logger().info('║  velocity_smoother                              ║')
        self.get_logger().info('║    → /cmd_vel  (Twist)                          ║')
        self.get_logger().info('║  emergency_stop_controller                      ║')
        self.get_logger().info('║    → /cmd_vel_safe  (Twist)                     ║')
        self.get_logger().info('║  arduino_driver                                 ║')
        self.get_logger().info('║    → /cmd_vel_debug (PWM L,R)                   ║')
        self.get_logger().info('║    → serial "L,R\\n" → Arduino → motors          ║')
        self.get_logger().info('╚══════════════════════════════════════════════════╝')
        self.get_logger().info('')

        # ── Readiness indicators ──
        self.create_subscription(
            OccupancyGrid, 'map', self._on_map, map_qos)
        self.create_subscription(
            Odometry, 'odom', self._on_odom, reliable)
        self.create_subscription(
            LaserScan, 'scan_raw', self._on_scan_raw,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))

        # ── Mission / exploration state ──
        self.create_subscription(
            String, 'mission_state', self._on_mission_state, reliable)
        self.create_subscription(
            String, 'exploration_state', self._on_exploration_state, reliable)
        self.create_subscription(
            String, 'exploration_control', self._on_exploration_control, reliable)
        self.create_subscription(
            Bool, 'exploration_complete', self._on_exploration_complete, reliable)

        # ── Velocity chain ──
        self.create_subscription(
            Twist, 'cmd_vel_nav', self._on_cmd_vel_nav, reliable)
        self.create_subscription(
            Twist, 'cmd_vel', self._on_cmd_vel, reliable)
        self.create_subscription(
            Twist, 'cmd_vel_safe', self._on_cmd_vel_safe, reliable)
        self.create_subscription(
            Twist, 'cmd_vel_debug', self._on_cmd_vel_debug, reliable)

        # ── Periodic readiness check (every 5 s) ──
        self._check_timer = self.create_timer(5.0, self._readiness_check)
        self._readiness_reported = False

    # ══════════════════════ Readiness ══════════════════════

    def _on_map(self, msg: OccupancyGrid):
        if not self._map_received:
            self._map_received = True
            w, h = msg.info.width, msg.info.height
            res = msg.info.resolution
            self.get_logger().info(
                f'✅ MAP RECEIVED  ({w}×{h}, res={res:.3f} m/cell)  '
                f'[publisher: async_slam_toolbox_node → /map]')

    def _on_odom(self, msg: Odometry):
        if not self._odom_received:
            self._odom_received = True
            self.get_logger().info(
                '✅ ODOM RECEIVED  '
                '[publisher: rf2o_laser_odometry → /odom, /tf (odom→base_link)]')

    def _on_scan_raw(self, msg: LaserScan):
        if not self._scan_received:
            self._scan_received = True
            n = len(msg.ranges)
            self.get_logger().info(
                f'✅ LIDAR READY  ({n} beams, range {msg.range_min:.2f}–{msg.range_max:.1f} m)  '
                f'[publisher: sllidar_node → /scan_raw]')

    def _readiness_check(self):
        if self._readiness_reported:
            return
        ready = self._map_received and self._odom_received and self._scan_received
        if ready:
            self._readiness_reported = True
            self.get_logger().info('')
            self.get_logger().info('🟢 ALL SYSTEMS READY — robot can move')
            self.get_logger().info('')
        else:
            missing = []
            if not self._scan_received:
                missing.append('/scan_raw (LiDAR)')
            if not self._odom_received:
                missing.append('/odom (rf2o)')
            if not self._map_received:
                missing.append('/map (SLAM)')
            self.get_logger().warn(
                f'⏳ Waiting for: {", ".join(missing)}')

    # ══════════════════════ Mission state ══════════════════════

    def _on_mission_state(self, msg: String):
        pass  # published every 1 s, too noisy

    def _on_exploration_state(self, msg: String):
        self.get_logger().info(
            f'🗺️  EXPLORATION STATE: {msg.data}  '
            f'[publisher: frontier_explorer → /exploration_state]')

    def _on_exploration_control(self, msg: String):
        self.get_logger().info(
            f'🎮 EXPLORATION CONTROL: "{msg.data}"  '
            f'[publisher: full_mission_controller → /exploration_control]  '
            f'[subscriber: frontier_explorer]')

    def _on_exploration_complete(self, msg: Bool):
        if msg.data:
            self.get_logger().info(
                '🏁 EXPLORATION COMPLETE  '
                '[publisher: frontier_explorer → /exploration_complete]  '
                '[subscriber: full_mission_controller]')

    # ══════════════════════ Velocity chain ══════════════════════

    def _on_cmd_vel_nav(self, msg: Twist):
        lin, ang = msg.linear.x, msg.angular.z
        if abs(lin) < 0.001 and abs(ang) < 0.001:
            return  # skip zero commands to reduce noise
        self.get_logger().info(
            f'⚡ /cmd_vel_nav  lin={lin:+.3f} ang={ang:+.3f}  '
            f'[publisher: controller_server]  '
            f'[subscribers: velocity_smoother, emergency_stop, arduino_driver]')

    def _on_cmd_vel(self, msg: Twist):
        lin, ang = msg.linear.x, msg.angular.z
        if abs(lin) < 0.001 and abs(ang) < 0.001:
            return
        self.get_logger().info(
            f'  ↳ /cmd_vel      lin={lin:+.3f} ang={ang:+.3f}  '
            f'[publisher: velocity_smoother]  '
            f'[subscriber: emergency_stop]')

    def _on_cmd_vel_safe(self, msg: Twist):
        lin, ang = msg.linear.x, msg.angular.z
        if abs(lin) < 0.001 and abs(ang) < 0.001:
            return
        self.get_logger().info(
            f'    ↳ /cmd_vel_safe lin={lin:+.3f} ang={ang:+.3f}  '
            f'[publisher: emergency_stop]  '
            f'[subscriber: arduino_driver → serial → motors]')

    def _on_cmd_vel_debug(self, msg: Twist):
        left_pwm = int(msg.linear.x)
        right_pwm = int(msg.linear.y)
        if left_pwm == 0 and right_pwm == 0:
            return
        self.get_logger().info(
            f'      ↳ ARDUINO   PWM L={left_pwm:+d} R={right_pwm:+d}  '
            f'[arduino_driver → serial "{left_pwm},{right_pwm}\\n" → motors]')


def main(args=None):
    rclpy.init(args=args)
    node = ScanMonitor()
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
