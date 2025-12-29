#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String


class CmdVelMux(Node):
    def __init__(self):
        super().__init__('cmd_vel_mux')

        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('input_timeout_sec', 0.5)
        self.declare_parameter('dock_latch_sec', 60.0)

        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.input_timeout_sec = float(self.get_parameter('input_timeout_sec').value)
        self.dock_latch_sec = float(self.get_parameter('dock_latch_sec').value)

        self.pub = self.create_publisher(Twist, '/cmd_vel_muxed', 10)

        self.last_explore_cmd = Twist()
        self.last_dock_cmd = Twist()
        self.last_explore_time = None
        self.last_dock_time = None

        self.docking_mode = False
        self.docking_mode_since = None

        self.create_subscription(Twist, '/cmd_vel_explore', self._explore_cb, 10)
        self.create_subscription(Twist, '/cmd_vel_dock', self._dock_cb, 10)
        self.create_subscription(Bool, '/start_docking', self._start_docking_cb, 10)
        self.create_subscription(String, '/dock_status', self._dock_status_cb, 10)

        period = 1.0 / max(1e-3, self.publish_rate_hz)
        self.create_timer(period, self._tick)

        self.get_logger().info(
            f"CmdVelMux started: rate={self.publish_rate_hz:.1f}Hz timeout={self.input_timeout_sec:.2f}s dock_latch={self.dock_latch_sec:.1f}s"
        )

    def _now(self):
        return self.get_clock().now()

    def _age_sec(self, t):
        if t is None:
            return float('inf')
        return (self._now() - t).nanoseconds / 1e9

    def _explore_cb(self, msg: Twist):
        self.last_explore_cmd = msg
        self.last_explore_time = self._now()

    def _dock_cb(self, msg: Twist):
        self.last_dock_cmd = msg
        self.last_dock_time = self._now()

    def _start_docking_cb(self, msg: Bool):
        if msg.data and not self.docking_mode:
            self.docking_mode = True
            self.docking_mode_since = self._now()
            self.get_logger().info('Docking mode latched')

    def _dock_status_cb(self, msg: String):
        if msg.data.strip().lower() == 'docked' and self.docking_mode:
            self.docking_mode = False
            self.docking_mode_since = None
            self.get_logger().info('Docking mode released (docked)')

    def _tick(self):
        cmd = Twist()

        if self.docking_mode:
            dock_age = self._age_sec(self.last_dock_time)
            dock_latch_age = self._age_sec(self.docking_mode_since)

            if dock_latch_age > self.dock_latch_sec:
                self.docking_mode = False
                self.docking_mode_since = None
                self.get_logger().warn('Docking mode timed out; releasing')
            elif dock_age <= self.input_timeout_sec:
                cmd = self.last_dock_cmd
            else:
                # Docking requested but docker isn't updating; keep stopped (don\'t fall back to explore).
                self.get_logger().warn(
                    f"Dock cmd stale ({dock_age:.2f}s); holding stop",
                    throttle_duration_sec=2.0,
                )
        else:
            explore_age = self._age_sec(self.last_explore_time)
            if explore_age <= self.input_timeout_sec:
                cmd = self.last_explore_cmd

        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMux()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
