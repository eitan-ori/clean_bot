#!/usr/bin/env python3
"""Throttle /scan_raw to /scan at a configurable rate.

Ensures all consumers (rf2o, SLAM, Nav2 costmaps) receive scans at
a rate the Pi 4 can process, so every scan gets a matching TF from rf2o.
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
        self.last_publish = 0.0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )
        self.pub = self.create_publisher(LaserScan, '/scan', qos)
        self.sub = self.create_subscription(
            LaserScan, '/scan_raw', self._on_scan, qos
        )
        self.get_logger().info(
            f'Throttling /scan_raw -> /scan at {rate:.1f} Hz'
        )

    def _on_scan(self, msg):
        now = time.monotonic()
        if now - self.last_publish >= self.period:
            self.pub.publish(msg)
            self.last_publish = now


def main(args=None):
    rclpy.init(args=args)
    node = ScanThrottle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
