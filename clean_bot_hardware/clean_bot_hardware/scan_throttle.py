#!/usr/bin/env python3
"""Lightweight scan throttle: /scan_raw -> /scan.

We used to sync scans to odom->base_link TF (rf2o-era). In the current setup
(Cartographer provides localization), the extra TF bookkeeping adds CPU load
and can create failure modes if TF is delayed.

This node now does the simplest, lowest-overhead thing:
- Subscribe to /scan_raw (BEST_EFFORT)
- Re-publish to /scan at a fixed max rate
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan


class ScanThrottle(Node):
    def __init__(self):
        super().__init__('scan_throttle')
        rate = self.declare_parameter('rate', 5.0).value
        self.period = 1.0 / max(rate, 0.1)
        self._last_publish_wall = 0.0

        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )
        self.pub = self.create_publisher(LaserScan, '/scan', pub_qos)
        self.create_subscription(LaserScan, '/scan_raw', self._on_scan, sub_qos)

        self.get_logger().info(f'Scan throttle /scan_raw -> /scan @ {rate:.1f} Hz')

    def _on_scan(self, msg):
        """Rate-limit and publish."""
        now_wall = time.monotonic()
        if now_wall - self._last_publish_wall < self.period:
            return
        self.pub.publish(msg)
        self._last_publish_wall = now_wall


def main(args=None):
    rclpy.init(args=args)
    node = ScanThrottle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
