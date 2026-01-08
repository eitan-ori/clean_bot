import unittest
from unittest.mock import MagicMock
import math
import sys
import os
import rclpy
from sensor_msgs.msg import LaserScan

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'scripts'))
from dock_detector import DockDetector  # noqa: E402


class TestDockDetector(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = DockDetector()
        self.node.publisher = MagicMock()

    def tearDown(self):
        self.node.destroy_node()

    def test_v_shape_detection(self):
        # Create a synthetic scan with a V-shape at index 180 (front)
        scan = LaserScan()
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = (2 * math.pi) / 360
        scan.ranges = [3.0] * 360

        # Create V-shape at index 180 (0 degrees)
        # Center is 1.0m away
        # Neighbors increase distance
        center_idx = 180
        scan.ranges[center_idx] = 1.0
        for i in range(1, 10):
            scan.ranges[center_idx - i] = 1.0 + (i * 0.02)
            scan.ranges[center_idx + i] = 1.0 + (i * 0.02)

        self.node.scan_callback(scan)

        # Check if publisher was called
        self.node.publisher.publish.assert_called_once()

        # Check pose
        pose = self.node.publisher.publish.call_args[0][0]
        self.assertAlmostEqual(pose.pose.position.x, 1.0, delta=0.1)
        self.assertAlmostEqual(pose.pose.position.y, 0.0, delta=0.1)

    def test_no_dock_far_away(self):
        scan = LaserScan()
        scan.ranges = [5.0] * 360  # All far away

        self.node.scan_callback(scan)
        self.node.publisher.publish.assert_not_called()


if __name__ == '__main__':
    unittest.main()
