import unittest
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import launch
import launch_ros
import launch_testing
import os
import time
import pytest
from ament_index_python.packages import get_package_share_directory

@pytest.mark.rostest
def generate_test_description():
    pkg_dir = get_package_share_directory('my_robot_slam')
    launch_file = os.path.join(pkg_dir, 'launch', 'exploration_mission.launch.py')

    return launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(launch_file)
        ),
        launch_testing.actions.ReadyToTest()
    ])

class TestExplorationMission(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_exploration_mission')
        self.dock_status = None
        self.sub = self.node.create_subscription(
            String,
            '/dock_status',
            self.status_callback,
            10
        )

    def tearDown(self):
        self.node.destroy_node()

    def status_callback(self, msg):
        self.dock_status = msg.data

    def test_mission_completion(self):
        start_time = time.time()
        timeout = 600.0 # Increased timeout
        
        while time.time() - start_time < timeout:
            rclpy.spin_once(self.node, timeout_sec=1.0)
            if self.dock_status == 'docked':
                self.node.get_logger().info("Mission Complete: Robot Docked!")
                return
        
        self.fail("Mission timed out! Robot did not dock.")
