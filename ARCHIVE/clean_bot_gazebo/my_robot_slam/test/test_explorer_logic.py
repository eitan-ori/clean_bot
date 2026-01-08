import unittest
from unittest.mock import MagicMock, patch
import sys
import os
import numpy as np
import math
import importlib

# Add scripts to path
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'scripts'))

# Define a dummy Node class to replace rclpy.node.Node
class DummyNode:
    def __init__(self, node_name):
        self.node_name = node_name
    def create_publisher(self, *args, **kwargs):
        return MagicMock()
    def create_subscription(self, *args, **kwargs):
        return MagicMock()
    def create_timer(self, *args, **kwargs):
        return MagicMock()
    def get_logger(self):
        return MagicMock()
    def get_clock(self):
        return MagicMock()
    def destroy_node(self):
        pass

class TestExplorerLogic(unittest.TestCase):
    def setUp(self):
        # Patch Node with DummyNode
        self.node_patcher = patch('rclpy.node.Node', DummyNode)
        self.node_patcher.start()
        
        # Import/reload the module to ensure it uses the DummyNode
        import autonomous_explorer
        importlib.reload(autonomous_explorer)
        self.module = autonomous_explorer
        
        self.explorer = self.module.AutonomousExplorer()
        
        # Setup basic map info
        self.explorer.map_info = MagicMock()
        self.explorer.map_info.resolution = 0.1
        self.explorer.map_info.width = 100
        self.explorer.map_info.height = 100
        self.explorer.map_info.origin.position.x = -5.0
        self.explorer.map_info.origin.position.y = -5.0
        
        # Setup map data (100x100)
        self.explorer.map_data = np.zeros((100, 100), dtype=np.int8)

    def tearDown(self):
        self.node_patcher.stop()
        
    def test_coordinate_conversion(self):
        # Test (0,0) world -> grid
        # Origin is (-5, -5). 0 - (-5) = 5. 5 / 0.1 = 50.
        gx, gy = self.explorer.world_to_grid(0.0, 0.0)
        self.assertEqual(gx, 50)
        self.assertEqual(gy, 50)
        
        # Test grid -> world
        wx, wy = self.explorer.grid_to_world(50, 50)
        self.assertAlmostEqual(wx, 0.05) # Center of cell
        self.assertAlmostEqual(wy, 0.05)

    def test_frontier_detection(self):
        # Create a known frontier
        # Fill map with -1 (unknown)
        self.explorer.map_data.fill(-1)
        
        # Clear a center area (free space)
        # 40-60 is free
        self.explorer.map_data[40:60, 40:60] = 0
        
        # The boundary between 0 and -1 is the frontier.
        # Let's set the robot position in the free space
        self.explorer.x = 0.0
        self.explorer.y = 0.0
        
        target = self.explorer.find_frontier()
        self.assertIsNotNone(target)
        tx, ty = target
        
        # Target should be roughly on the boundary of the 2x2m box (indices 40 or 60)
        # 40 -> -1.0m, 60 -> 1.0m
        # Distance should be around 1.0m
        dist = math.hypot(tx, ty)
        self.assertTrue(0.9 < dist < 1.5, f"Frontier dist {dist} not in expected range")

    def test_astar_planning(self):
        # Create a simple obstacle
        # Start (50, 50), Goal (50, 60)
        # Obstacle at (50, 55)
        
        self.explorer.map_data.fill(0) # All free
        
        # Block (48-52, 55)
        self.explorer.map_data[55, 48:53] = 100
        
        start = (50, 50)
        goal = (50, 60)
        
        # Create blocked mask
        blocked = (self.explorer.map_data == 100)
        
        path = self.explorer.astar(start, goal, blocked)
        
        self.assertIsNotNone(path)
        # Path length should be >= 10 (straight line)
        self.assertTrue(len(path) >= 10)
        
        # Check that path doesn't go through obstacle
        for (px, py) in path:
            self.assertFalse(blocked[py, px], f"Path hit obstacle at {px},{py}")

    def test_return_home_logic(self):
        self.explorer.state = "RETURNING"
        self.explorer.home_x = -2.0
        self.explorer.home_y = -2.0
        self.explorer.x = 0.0
        self.explorer.y = 0.0
        self.explorer.yaw = 0.0
        
        # Mock map for planning
        self.explorer.map_data.fill(0)
        
        # Mock check_collision_ahead to return False
        self.explorer.check_collision_ahead = MagicMock(return_value=False)
        self.explorer.get_clearances = MagicMock(return_value=(10.0, 10.0, 10.0))
        
        # Verify plan_path_to_target is called with home coordinates
        # We can mock astar to return a dummy path
        with patch.object(self.explorer, 'astar', return_value=[(50,50), (40,40)]) as mock_astar:
             path = self.explorer.plan_path_to_target(self.explorer.home_x, self.explorer.home_y)
             self.assertIsNotNone(path)
             # Check if astar was called
             mock_astar.assert_called()

if __name__ == '__main__':
    unittest.main()
