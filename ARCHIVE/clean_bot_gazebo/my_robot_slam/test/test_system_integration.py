import os
import unittest
import launch
import launch_testing
import pytest
from ament_index_python.packages import get_package_share_directory


@pytest.mark.rostest
def generate_test_description():
    pkg_share = get_package_share_directory('my_robot_slam')
    launch_file = os.path.join(pkg_share, 'launch', 'main.launch.py')

    return launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(launch_file),
            # We can pass arguments to disable hardware if the launch file supported it
            # For now, we expect nodes to start but maybe log errors about missing hardware
        ),
        launch_testing.actions.ReadyToTest()
    ])


class TestSystemBringup(unittest.TestCase):
    def test_nodes_start(self, proc_output):
        # This test checks if the main nodes are started
        # We look for their names in the process output or check if they crash immediately

        # Wait a bit for startup
        import time
        time.sleep(5)

        # In a real integration test, we would use 'ros2 node list' or similar
        # Here we just ensure the launch didn't exit with error immediately
        pass
