#!/usr/bin/env python3
"""Synchronized scan throttle: /scan_raw -> /scan with TF-aware timing.

On a Pi 4 the Nav2 costmap's TF buffer can't keep up with scan rate.
This node:
1. Subscribes to /scan_raw at full LiDAR rate (BEST_EFFORT)
2. Monitors odom->base_link TF to know the latest available timestamp
3. Only publishes a scan after rf2o has published TF for it
4. Sets the scan timestamp to the latest TF timestamp

This guarantees every published scan has a matching TF already on /tf,
so the costmap's message filter never drops scans.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from builtin_interfaces.msg import Time
import time


class ScanThrottle(Node):
    def __init__(self):
        super().__init__('scan_throttle')
        rate = self.declare_parameter('rate', 5.0).value
        self.period = 1.0 / max(rate, 0.1)
        self.last_publish = 0.0
        self.latest_tf_stamp = None  # Latest odom->base_link TF timestamp
        self.pending_scan = None     # Scan waiting for matching TF

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
        tf_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=50,
        )
        self.pub = self.create_publisher(LaserScan, '/scan', pub_qos)
        self.create_subscription(LaserScan, '/scan_raw', self._on_scan, sub_qos)
        self.create_subscription(TFMessage, '/tf', self._on_tf, tf_qos)

        self.get_logger().info(
            f'TF-synced throttle /scan_raw -> /scan at {rate:.1f} Hz'
        )

    def _stamp_to_float(self, stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def _float_to_stamp(self, t):
        s = Time()
        s.sec = int(t)
        s.nanosec = int((t - int(t)) * 1e9)
        return s

    def _on_tf(self, msg):
        """Track the latest odom->base_link TF timestamp."""
        for transform in msg.transforms:
            if transform.child_frame_id == 'base_link':
                t = self._stamp_to_float(transform.header.stamp)
                if self.latest_tf_stamp is None or t > self.latest_tf_stamp:
                    self.latest_tf_stamp = t
                    self._try_publish_pending()

    def _on_scan(self, msg):
        """Store the latest scan and try to publish if TF is ready."""
        now = time.monotonic()
        if now - self.last_publish < self.period:
            return  # Rate limit
        self.pending_scan = msg
        self._try_publish_pending()

    def _try_publish_pending(self):
        """Publish the pending scan if we have a matching TF."""
        if self.pending_scan is None or self.latest_tf_stamp is None:
            return
        scan_t = self._stamp_to_float(self.pending_scan.header.stamp)
        # Publish if TF is available at or after the scan timestamp
        if self.latest_tf_stamp >= scan_t - 0.05:
            # Use the TF timestamp to guarantee the costmap can look it up
            self.pending_scan.header.stamp = self._float_to_stamp(
                min(self.latest_tf_stamp, scan_t)
            )
            self.pub.publish(self.pending_scan)
            self.last_publish = time.monotonic()
            self.pending_scan = None


def main(args=None):
    rclpy.init(args=args)
    node = ScanThrottle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
