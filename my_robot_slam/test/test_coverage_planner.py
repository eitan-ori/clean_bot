import unittest
import rclpy
from nav_msgs.msg import OccupancyGrid

import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'scripts'))

from coverage_planner import CoveragePlanner


class TestCoveragePlanner(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = CoveragePlanner()

    def tearDown(self):
        self.node.destroy_node()

    def test_path_generation(self):
        # Create a dummy map
        grid = OccupancyGrid()
        grid.info.resolution = 0.1
        grid.info.width = 100  # 10 meters
        grid.info.height = 100  # 10 meters
        grid.info.origin.position.x = 0.0
        grid.info.origin.position.y = 0.0

        self.node.map_data = grid
        self.node.robot_width = 1.0  # Large width for fewer points

        self.node.generate_coverage_path()

        # Check if goals were generated
        self.assertTrue(len(self.node.goals) > 0)

        # Check bounds
        for x, y in self.node.goals:
            self.assertGreaterEqual(x, 0.0)
            self.assertLessEqual(x, 10.0)
            self.assertGreaterEqual(y, 0.0)
            self.assertLessEqual(y, 10.0)


if __name__ == '__main__':
    unittest.main()
