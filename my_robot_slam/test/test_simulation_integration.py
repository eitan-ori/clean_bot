import os
import pytest
import unittest
import rclpy
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest
from ament_index_python.packages import get_package_share_directory


@pytest.mark.launch_test
def generate_test_description():
    pkg_share = get_package_share_directory('my_robot_slam')
    launch_file = os.path.join(pkg_share, 'launch', 'docking_simulation.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_file)
        ),
        ReadyToTest()
    ])


class TestDockingSimulation(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_docking_sim')
        self.docked = False
        self.final_x = 0.0
        self.final_y = 0.0

    def tearDown(self):
        self.node.destroy_node()

    def status_callback(self, msg):
        if msg.data == 'docked':
            self.docked = True

    def odom_callback(self, msg):
        self.final_x = msg.pose.pose.position.x
        self.final_y = msg.pose.pose.position.y

    def test_docking_process(self):
        # Subscribe to status and odom
        self.node.create_subscription(String, '/dock_status', self.status_callback, 10)
        self.node.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Publisher to start docking
        start_pub = self.node.create_publisher(Bool, '/start_docking', 10)

        # Wait a bit for connections
        import time
        time.sleep(2.0)
        
        # Publish start signal
        msg = Bool()
        msg.data = True
        start_pub.publish(msg)
        self.node.get_logger().info("Published start_docking signal")

        # Wait for docking to complete (timeout 60s)
        start_time = self.node.get_clock().now()
        while not self.docked:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if (self.node.get_clock().now() - start_time).nanoseconds > 60e9:
                self.fail("Docking timed out")

        # Verify position
        # Dock is at (1.5, 0). Robot stops 0.3m away -> (1.2, 0)
        # Note: The simulation might not be perfectly accurate, so we use a larger delta
        self.assertAlmostEqual(self.final_x, 1.2, delta=0.3)
        self.assertAlmostEqual(self.final_y, 0.0, delta=0.3)

    def status_callback(self, msg):
        if msg.data == 'docked':
            self.docked = True

    def odom_callback(self, msg):
        self.final_x = msg.pose.pose.position.x
        self.final_y = msg.pose.pose.position.y
