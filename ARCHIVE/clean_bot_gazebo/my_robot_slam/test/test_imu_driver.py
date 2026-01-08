import unittest
from unittest.mock import MagicMock, patch
from sensor_msgs.msg import Imu

import sys
import os

sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'scripts'))
import importlib


class TestImuDriver(unittest.TestCase):
    ImuDriver = None

    @classmethod
    def setUpClass(cls):
        from builtin_interfaces.msg import Time

        class _DummyNow:
            def to_msg(self):
                return Time(sec=0, nanosec=0)

        class _DummyClock:
            def now(self):
                return _DummyNow()

        class DummyNode:
            def __init__(self, node_name):
                self._params = {}
                self._clock = _DummyClock()

            def declare_parameter(self, name, default_value=None):
                self._params[name] = default_value

            def get_parameter(self, name):
                class _P:
                    def __init__(self, v):
                        self.value = v

                return _P(self._params.get(name))

            def create_publisher(self, *args, **kwargs):
                return MagicMock()

            def create_timer(self, *args, **kwargs):
                return MagicMock()

            def get_clock(self):
                return self._clock

            def get_logger(self):
                return MagicMock()

            def destroy_node(self):
                pass

        # Patch rclpy.node.Node so ImuDriver can be instantiated without ROS
        cls._node_patcher = patch('rclpy.node.Node', DummyNode)
        cls._node_patcher.start()

        with patch.dict('sys.modules', {'smbus2': MagicMock()}):
            import imu_driver
            importlib.reload(imu_driver)
            cls.ImuDriver = imu_driver.ImuDriver

    @classmethod
    def tearDownClass(cls):
        cls._node_patcher.stop()

    def setUp(self):
        self.node = self.ImuDriver()

    def tearDown(self):
        self.node.destroy_node()

    def test_publish_imu(self):
        # Mock the read_raw_data method to return known values
        self.node.read_raw_data = MagicMock(return_value=(0.1, 0.2, 9.8, 0.01, 0.02, 0.03))

        # Trigger publication
        self.node.publish_imu()

        # Verify publish happened
        self.node.imu_pub.publish.assert_called_once()
        published = self.node.imu_pub.publish.call_args[0][0]
        self.assertIsInstance(published, Imu)
        self.assertAlmostEqual(published.linear_acceleration.z, 9.8)

        # Direct check of logic
        ax, ay, az, gx, gy, gz = self.node.read_raw_data()
        self.assertEqual(az, 9.8)


if __name__ == '__main__':
    unittest.main()
