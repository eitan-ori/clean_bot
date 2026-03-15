#!/usr/bin/env python3
"""TF-synchronized scan throttle: /scan_raw -> /scan.

On a Pi 4, Nav2 costmaps run in a single-threaded executor that
can starve TF listener callbacks.  This node:

1. Subscribes to /scan_raw (BEST_EFFORT, from LiDAR) and rate-limits.
2. Monitors /tf for the latest odom->base_link timestamp (from rf2o,
   which subscribes to /scan_raw independently — no circular dependency).
3. Only publishes a scan AFTER the matching TF is on the wire,
   backdated by BACKDATE seconds so the costmap's (possibly stale)
   TF buffer can always satisfy the lookup.
4. Falls back after FALLBACK_TIMEOUT seconds to avoid total stall.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from builtin_interfaces.msg import Time
import time


# How far to backdate scan timestamps (seconds).
# The costmap looks up TF at this earlier time — even a stale TF buffer
# will have data 0.5 s in the past.  Spatial error at 0.15 m/s ≈ 7.5 cm,
# well within the 35 cm inflation radius.
BACKDATE = 0.5

# If no matching TF arrives within this many seconds after a scan is
# stored, publish it anyway (with the latest known TF stamp minus
# BACKDATE) so SLAM and costmaps are not starved entirely.
FALLBACK_TIMEOUT = 2.0


class ScanThrottle(Node):
    def __init__(self):
        super().__init__('scan_throttle')
        rate = self.declare_parameter('rate', 5.0).value
        self.period = 1.0 / max(rate, 0.1)
        self.last_publish = 0.0

        self.latest_tf_stamp = None   # Latest odom->base_link ROS timestamp
        self.pending_scan = None      # Scan waiting for matching TF
        self.pending_wall = None      # Wall-clock time scan was stored

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
            depth=100,
        )

        self.pub = self.create_publisher(LaserScan, '/scan', pub_qos)
        self.create_subscription(LaserScan, '/scan_raw', self._on_scan, sub_qos)
        self.create_subscription(TFMessage, '/tf', self._on_tf, tf_qos)

        # Periodic check for fallback timeout
        self.create_timer(0.5, self._fallback_check)

        self.get_logger().info(
            f'TF-synced throttle /scan_raw -> /scan @ {rate:.1f} Hz '
            f'(backdate={BACKDATE}s, fallback={FALLBACK_TIMEOUT}s)'
        )

    # ----- helpers -----

    @staticmethod
    def _stamp_to_float(stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    @staticmethod
    def _float_to_stamp(t):
        t = max(t, 0.0)
        s = Time()
        s.sec = int(t)
        s.nanosec = int((t - int(t)) * 1e9)
        return s

    # ----- callbacks -----

    def _on_tf(self, msg):
        """Track the latest odom->base_link TF timestamp."""
        for tf in msg.transforms:
            if tf.child_frame_id == 'base_link':
                t = self._stamp_to_float(tf.header.stamp)
                if self.latest_tf_stamp is None or t > self.latest_tf_stamp:
                    self.latest_tf_stamp = t
                    self._try_publish()

    def _on_scan(self, msg):
        """Rate-limit and store the latest scan."""
        now = time.monotonic()
        if now - self.last_publish < self.period:
            return
        self.pending_scan = msg
        self.pending_wall = now
        self._try_publish()

    def _fallback_check(self):
        """Publish stale pending scan so SLAM/costmaps are never completely starved."""
        if self.pending_scan is not None and self.pending_wall is not None:
            if time.monotonic() - self.pending_wall > FALLBACK_TIMEOUT:
                self._force_publish()

    # ----- publish logic -----

    def _try_publish(self):
        """Publish pending scan if matching TF is available."""
        if self.pending_scan is None or self.latest_tf_stamp is None:
            return
        scan_t = self._stamp_to_float(self.pending_scan.header.stamp)
        # TF must be at least as recent as the scan (with 0.15 s tolerance
        # for rf2o processing delay)
        if self.latest_tf_stamp >= scan_t - 0.15:
            safe_stamp = self.latest_tf_stamp - BACKDATE
            self.pending_scan.header.stamp = self._float_to_stamp(safe_stamp)
            self.pub.publish(self.pending_scan)
            self.last_publish = time.monotonic()
            self.pending_scan = None
            self.pending_wall = None

    def _force_publish(self):
        """Publish regardless of TF state (fallback)."""
        if self.pending_scan is None:
            return
        if self.latest_tf_stamp is not None:
            safe_stamp = self.latest_tf_stamp - BACKDATE
            self.pending_scan.header.stamp = self._float_to_stamp(safe_stamp)
        # else: keep original timestamp — best we can do
        self.pub.publish(self.pending_scan)
        self.last_publish = time.monotonic()
        self.pending_scan = None
        self.pending_wall = None
        self.get_logger().warn(
            'Published scan after TF timeout — check rf2o and LiDAR',
            throttle_duration_sec=10.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = ScanThrottle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
