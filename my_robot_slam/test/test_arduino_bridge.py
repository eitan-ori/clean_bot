import unittest
from unittest.mock import MagicMock, patch
import rclpy
from geometry_msgs.msg import Twist

import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'scripts'))

from arduino_bridge import ArduinoBridge


class TestArduinoBridge(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        # Mock serial during node construction so it doesn't try to open a real port
        self._serial_patcher = patch('arduino_bridge.serial.Serial', return_value=MagicMock())
        self._serial_patcher.start()
        self.node = ArduinoBridge()
        self.node.ser = MagicMock()  # Ensure serial is mocked

    def tearDown(self):
        self.node.destroy_node()
        self._serial_patcher.stop()

    def test_cmd_vel_callback(self):
        msg = Twist()
        msg.linear.x = 0.5
        msg.angular.z = 0.0

        self.node.cmd_vel_callback(msg)

        # Check if serial write was called with expected format
        # v_left = 0.5, v_right = 0.5
        expected_call = b'0.500,0.500\n'
        self.node.ser.write.assert_called_with(expected_call)

    def test_odometry_update(self):
        # Mock serial reading
        self.node.ser.in_waiting = True
        self.node.ser.readline.return_value = b'1000,1000,150.0\n'

        # First run initializes previous ticks
        self.node.update_odometry()
        self.assertEqual(self.node.left_ticks_prev, 1000)

        # Second run with movement
        self.node.ser.readline.return_value = b'2000,2000,140.0\n'
        self.node.update_odometry()

        # Check if position updated (moved forward)
        self.assertGreater(self.node.x, 0.0)
        self.assertEqual(self.node.y, 0.0)

    def test_ultrasonic_publish(self):
        # Mock serial reading with ultrasonic data
        self.node.ser.in_waiting = True
        self.node.ser.readline.return_value = b'1000,1000,125.5\n'

        # Mock publisher
        self.node.range_pub = MagicMock()

        # First run initializes previous ticks (no publish)
        self.node.update_odometry()

        # Second run should publish
        self.node.ser.readline.return_value = b'1100,1100,125.5\n'
        self.node.update_odometry()

        # Check if range message was published
        self.node.range_pub.publish.assert_called_once()
        msg = self.node.range_pub.publish.call_args[0][0]
        self.assertAlmostEqual(msg.range, 1.255)
        self.assertEqual(msg.header.frame_id, 'ultrasonic_link')


if __name__ == '__main__':
    unittest.main()
