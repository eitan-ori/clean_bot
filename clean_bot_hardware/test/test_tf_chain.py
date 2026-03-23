#!/usr/bin/env python3

from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import pytest

from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Imu


def test_cartographer_lua_tf_settings_are_consistent():
    """Guards against the most common TF-chain regression.

    If rf2o is disabled, Cartographer must provide odom->base_link.
    """

    pkg_root = Path(__file__).resolve().parents[1]
    carto_lua = pkg_root / 'config' / 'cartographer.lua'
    text = carto_lua.read_text(encoding='utf-8')

    assert 'published_frame = "base_link"' in text
    assert 'odom_frame = "odom"' in text
    assert 'provide_odom_frame = true' in text


def test_robot_bringup_rf2o_is_disabled_by_default_and_wired_correctly():
    """Guards against re-introducing rf2o / odom TF conflicts.

    Expected behavior:
    - rf2o is OFF by default (use_rf2o:=false)
    - rf2o launch is conditional on use_rf2o
    - tf_odom_publisher is only enabled when rf2o is disabled
    """

    pkg_root = Path(__file__).resolve().parents[1]
    bringup = pkg_root / 'launch' / 'robot_bringup.launch.py'
    text = bringup.read_text(encoding='utf-8')

    assert "LaunchConfiguration('use_rf2o', default='false')" in text
    assert "DeclareLaunchArgument('use_rf2o', default_value='false'" in text
    assert 'condition=IfCondition(use_rf2o)' in text
    assert 'condition=UnlessCondition(use_rf2o)' in text


def test_imu_odom_broadcaster_builds_expected_transform_without_ros_init():
    """Unit-test the callback logic without spinning ROS.

    This catches frame_id mistakes that break the TF chain.
    """

    from clean_bot_hardware.imu_odom_broadcaster import ImuOdomBroadcaster

    captured = {}

    class StubBroadcaster:
        def sendTransform(self, t):
            captured['t'] = t

    dummy = SimpleNamespace(
        parent_frame='odom',
        child_frame='base_link',
        tf_broadcaster=StubBroadcaster(),
    )

    msg = Imu()
    msg.header.frame_id = 'imu_link'
    msg.header.stamp = TimeMsg(sec=123, nanosec=456)
    msg.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

    # Call the method as an unbound function.
    ImuOdomBroadcaster.imu_callback(dummy, msg)

    t = captured.get('t')
    assert t is not None
    assert t.header.frame_id == 'odom'
    assert t.child_frame_id == 'base_link'
    assert t.header.stamp.sec == 123
    assert t.header.stamp.nanosec == 456

    assert t.transform.translation.x == pytest.approx(0.0)
    assert t.transform.translation.y == pytest.approx(0.0)
    assert t.transform.translation.z == pytest.approx(0.0)

    assert t.transform.rotation.w == pytest.approx(1.0)
    assert t.transform.rotation.x == pytest.approx(0.0)
    assert t.transform.rotation.y == pytest.approx(0.0)
    assert t.transform.rotation.z == pytest.approx(0.0)


def test_tf_odom_publisher_quat_to_yaw_identity():
    from clean_bot_hardware.tf_odom_publisher import _quat_to_yaw

    q = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    assert _quat_to_yaw(q) == pytest.approx(0.0, abs=1e-10)
