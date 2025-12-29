import os
import pytest
import unittest

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest
from ament_index_python.packages import get_package_share_directory

@pytest.mark.launch_test
def generate_test_description():
    sim_launch_path = os.path.join(
        get_package_share_directory('clean_bot_gazebo'),
        'launch',
        'sim.launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim_launch_path),
            launch_arguments={'gui': 'false'}.items(),
        ),
        ReadyToTest()
    ])

class TestSimLaunch(unittest.TestCase):
    def test_sim_starts(self, proc_output):
        # This is a very basic test just to see if it launches without immediate failure
        pass
