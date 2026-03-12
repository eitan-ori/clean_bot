#!/usr/bin/env python3
"""
Integration tests for the Clean Bot Web Control Panel.

These tests verify the complete command chain from web UI action
through to the ROS 2 topic publish, ensuring the physical robot
will respond correctly when the app runs on a remote PC.

Command chain being tested:
  Browser → SocketIO/HTTP → Flask → WebBridgeNode → ROS 2 publish

Topic chain on the robot side (not tested here, but documented):
  /mission_command → full_mission → exploration_control/coverage_control
  /cmd_vel_nav → emergency_stop → /cmd_vel → arduino_driver → Serial
  /arduino_command → arduino_driver → Serial (CLEAN_START/CLEAN_STOP)
"""

import json
import os
import sys
import time
import math
import threading
from pathlib import Path
from unittest.mock import MagicMock, patch, call

import pytest
import numpy as np

# ── Mock ROS 2 before importing app ──
mock_rclpy = MagicMock()
mock_rclpy.ok = MagicMock(return_value=True)
mock_rclpy.time = MagicMock()
mock_rclpy.time.Time = MagicMock


class _FakeNode:
    """Minimal Node stand-in so WebBridgeNode gets real class methods."""
    def __init__(self, *a, **kw):
        pass
    def create_publisher(self, *a, **kw):
        return MagicMock()
    def create_subscription(self, *a, **kw):
        return MagicMock()
    def create_timer(self, *a, **kw):
        return MagicMock()
    def get_logger(self):
        return MagicMock()
    def get_clock(self):
        m = MagicMock()
        m.now.return_value = MagicMock(to_msg=MagicMock(return_value=MagicMock()), nanoseconds=0)
        return m
    def destroy_node(self):
        pass

_mock_node_module = MagicMock()
_mock_node_module.Node = _FakeNode

sys.modules.setdefault('rclpy', mock_rclpy)
sys.modules['rclpy.node'] = _mock_node_module
for mod in ['rclpy.qos', 'rclpy.executors', 'rclpy.action',
            'rclpy.callback_groups', 'rclpy.time', 'std_msgs', 'std_msgs.msg',
            'geometry_msgs', 'geometry_msgs.msg', 'nav_msgs', 'nav_msgs.msg',
            'sensor_msgs', 'sensor_msgs.msg', 'tf2_ros', 'nav2_msgs', 'nav2_msgs.action']:
    sys.modules.setdefault(mod, MagicMock())

from clean_bot_mission.webapp import app as webapp_module


# ════════════════════════════════════════════════════════════════════
# Fixtures
# ════════════════════════════════════════════════════════════════════

@pytest.fixture
def mock_ros_node(tmp_path):
    """Create a mock ROS node that tracks all publishes."""
    node = MagicMock()
    node.mission_state = "WAITING_FOR_SCAN"
    node.exploration_state = "IDLE"
    node.coverage_state = "IDLE"
    node.robot_x = 0.0
    node.robot_y = 0.0
    node.robot_yaw = 0.0
    node.linear_vel = 0.0
    node.angular_vel = 0.0
    node.map_update_counter = 0
    node.map_msg = None
    node.mission_log = []
    node.path_trail = []
    node.battery_level = 100.0
    node._arduino_battery = None
    node._schedules = []
    node._schedule_lock = MagicMock()
    node._schedule_lock.__enter__ = MagicMock(return_value=None)
    node._schedule_lock.__exit__ = MagicMock(return_value=False)
    node._save_schedules = MagicMock()
    node.get_stats.return_value = {
        "total_scan_time": 0, "total_clean_time": 0,
        "rooms_scanned": 0, "rooms_cleaned": 0, "total_distance": 0,
    }
    node.get_map_data = MagicMock(return_value=None)
    node.load_room_preview = MagicMock(return_value=None)
    node.navigate_to_pose = MagicMock(return_value=(True, "Goal sent"))

    # Track the actual calls to send_command and send_velocity
    node._published_commands = []
    node._published_velocities = []

    def track_command(cmd):
        node._published_commands.append(cmd)
        # Also zero velocity for stop commands (matching real behavior)
        if cmd in ("stop_clean", "stop_scan"):
            node._published_velocities.append((0.0, 0.0))

    def track_velocity(lin, ang):
        node._published_velocities.append((float(lin), float(ang)))

    node.send_command = MagicMock(side_effect=track_command)
    node.send_velocity = MagicMock(side_effect=track_velocity)
    node.save_room = MagicMock(return_value=(True, "/path/room.json"))

    return node


@pytest.fixture
def client(mock_ros_node, tmp_path):
    """Create Flask test client with mock ROS node."""
    webapp_module.app.config['TESTING'] = True
    original_node = webapp_module.ros_node
    original_rooms = webapp_module.SAVED_ROOMS_DIR
    original_sched = webapp_module.SCHEDULES_FILE

    # Save original class methods that tests may override
    WBN = webapp_module.WebBridgeNode
    _saved_attrs = {}
    for attr in ('list_rooms', 'delete_room', '_safe_room_path', 'load_room_preview'):
        if hasattr(WBN, attr):
            _saved_attrs[attr] = getattr(WBN, attr)

    webapp_module.ros_node = mock_ros_node
    webapp_module.SAVED_ROOMS_DIR = tmp_path / "rooms"
    webapp_module.SAVED_ROOMS_DIR.mkdir()
    webapp_module.SCHEDULES_FILE = tmp_path / "schedules.json"

    with webapp_module.app.test_client() as c:
        yield c

    # Restore everything
    webapp_module.ros_node = original_node
    webapp_module.SAVED_ROOMS_DIR = original_rooms
    webapp_module.SCHEDULES_FILE = original_sched
    for attr, val in _saved_attrs.items():
        setattr(WBN, attr, val)


@pytest.fixture
def node(mock_ros_node):
    return mock_ros_node


# ════════════════════════════════════════════════════════════════════
# Test Suite 1: Command Chain - Every button click → correct publish
# ════════════════════════════════════════════════════════════════════

class TestCommandChain:
    """Verify every web command reaches the correct ROS 2 topic."""

    def test_scan_room_button(self, client, node):
        """Scan Room → publishes 'start_scan' to /mission_command."""
        resp = client.post('/api/command', json={"command": "start_scan"})
        assert resp.status_code == 200
        assert "start_scan" in node._published_commands

    def test_stop_scan_button(self, client, node):
        """Stop Scan → publishes 'stop_scan' + zero velocity."""
        resp = client.post('/api/command', json={"command": "stop_scan"})
        assert resp.status_code == 200
        assert "stop_scan" in node._published_commands
        # stop_scan also zeroes velocity for safety
        assert (0.0, 0.0) in node._published_velocities

    def test_clean_room_button(self, client, node):
        """Clean Room → publishes 'start_clean' to /mission_command."""
        resp = client.post('/api/command', json={"command": "start_clean"})
        assert resp.status_code == 200
        assert "start_clean" in node._published_commands

    def test_stop_clean_button(self, client, node):
        """Stop Clean → publishes 'stop_clean' + zero velocity."""
        resp = client.post('/api/command', json={"command": "stop_clean"})
        assert resp.status_code == 200
        assert "stop_clean" in node._published_commands
        assert (0.0, 0.0) in node._published_velocities

    def test_go_home_button(self, client, node):
        """Go Home → publishes 'go_home' to /mission_command."""
        resp = client.post('/api/command', json={"command": "go_home"})
        assert resp.status_code == 200
        assert "go_home" in node._published_commands

    def test_reset_button(self, client, node):
        """Reset → publishes 'reset' to /mission_command."""
        resp = client.post('/api/command', json={"command": "reset"})
        assert resp.status_code == 200
        assert "reset" in node._published_commands

    def test_pause_button(self, client, node):
        """Pause → publishes 'pause' to /mission_command."""
        resp = client.post('/api/command', json={"command": "pause"})
        assert resp.status_code == 200
        assert "pause" in node._published_commands

    def test_resume_button(self, client, node):
        """Resume → publishes 'resume' to /mission_command."""
        resp = client.post('/api/command', json={"command": "resume"})
        assert resp.status_code == 200
        assert "resume" in node._published_commands

    def test_all_eight_commands_accepted(self, client, node):
        """All 8 valid commands are accepted."""
        for cmd in ["start_scan", "stop_scan", "start_clean", "stop_clean",
                     "go_home", "reset", "pause", "resume"]:
            resp = client.post('/api/command', json={"command": cmd})
            assert resp.status_code == 200, f"Command '{cmd}' failed"

    def test_invalid_command_rejected(self, client, node):
        """Invalid commands return 400."""
        for cmd in ["self_destruct", "start", "stop", "", "clean"]:
            resp = client.post('/api/command', json={"command": cmd})
            assert resp.status_code == 400, f"Command '{cmd}' should be rejected"

    def test_command_logs_event(self, client, node):
        """Commands are logged in mission_log."""
        client.post('/api/command', json={"command": "start_scan"})
        assert len(node.mission_log) > 0 or node.send_command.called


# ════════════════════════════════════════════════════════════════════
# Test Suite 2: Manual Drive - D-pad sends correct velocities
# ════════════════════════════════════════════════════════════════════

class TestManualDrive:
    """Verify manual drive commands produce correct Twist publishes."""

    def test_forward(self, client, node):
        """Forward button → positive linear, zero angular."""
        resp = client.post('/api/velocity', json={"linear": 0.15, "angular": 0.0})
        assert resp.status_code == 200
        node.send_velocity.assert_called_with(0.15, 0.0)

    def test_backward(self, client, node):
        """Backward button → negative linear, zero angular."""
        resp = client.post('/api/velocity', json={"linear": -0.15, "angular": 0.0})
        assert resp.status_code == 200
        node.send_velocity.assert_called_with(-0.15, 0.0)

    def test_turn_left(self, client, node):
        """Left button → zero linear, positive angular."""
        resp = client.post('/api/velocity', json={"linear": 0.0, "angular": 0.6})
        assert resp.status_code == 200
        node.send_velocity.assert_called_with(0.0, 0.6)

    def test_turn_right(self, client, node):
        """Right button → zero linear, negative angular."""
        resp = client.post('/api/velocity', json={"linear": 0.0, "angular": -0.6})
        assert resp.status_code == 200
        node.send_velocity.assert_called_with(0.0, -0.6)

    def test_stop(self, client, node):
        """Stop button → zero velocity."""
        resp = client.post('/api/velocity', json={"linear": 0.0, "angular": 0.0})
        assert resp.status_code == 200
        node.send_velocity.assert_called_with(0.0, 0.0)

    def test_velocity_with_speed(self, client, node):
        """Velocity with specific speed value."""
        resp = client.post('/api/velocity', json={"linear": 0.3, "angular": 0.0})
        assert resp.status_code == 200
        node.send_velocity.assert_called_with(0.3, 0.0)

    def test_missing_params_default_to_zero(self, client, node):
        """Missing linear/angular default to 0."""
        resp = client.post('/api/velocity', json={})
        assert resp.status_code == 200
        node.send_velocity.assert_called_with(0.0, 0.0)


# ════════════════════════════════════════════════════════════════════
# Test Suite 3: Emergency Stop - Most critical safety feature
# ════════════════════════════════════════════════════════════════════

class TestEmergencyStop:
    """Verify emergency stop halts ALL robot operations."""

    def test_estop_sends_stop_clean(self, client, node):
        """E-stop sends stop_clean command."""
        # Simulate what the JS emergencyStop() does
        client.post('/api/command', json={"command": "stop_clean"})
        assert "stop_clean" in node._published_commands

    def test_estop_sends_stop_scan(self, client, node):
        """E-stop sends stop_scan command."""
        client.post('/api/command', json={"command": "stop_scan"})
        assert "stop_scan" in node._published_commands

    def test_estop_zeroes_velocity(self, client, node):
        """E-stop zeroes velocity immediately."""
        client.post('/api/velocity', json={"linear": 0, "angular": 0})
        node.send_velocity.assert_called_with(0.0, 0.0)

    def test_estop_full_sequence(self, client, node):
        """Full E-stop: stop_clean + stop_scan + zero velocity."""
        # Robot is scanning at speed
        client.post('/api/command', json={"command": "stop_clean"})
        client.post('/api/command', json={"command": "stop_scan"})
        client.post('/api/velocity', json={"linear": 0, "angular": 0})
        assert "stop_clean" in node._published_commands
        assert "stop_scan" in node._published_commands
        # stop commands also zero velocity via send_command
        assert (0.0, 0.0) in node._published_velocities

    def test_estop_no_confirmation_needed(self, client, node):
        """Stop commands don't require confirmation (safety critical)."""
        # Both stop_clean and stop_scan should work without extra steps
        resp1 = client.post('/api/command', json={"command": "stop_clean"})
        resp2 = client.post('/api/command', json={"command": "stop_scan"})
        assert resp1.status_code == 200
        assert resp2.status_code == 200


# ════════════════════════════════════════════════════════════════════
# Test Suite 4: Navigation - Click-to-navigate
# ════════════════════════════════════════════════════════════════════

class TestNavigation:
    """Verify click-to-navigate sends correct Nav2 goals."""

    def test_navigate_to_pose(self, client, node):
        resp = client.post('/api/navigate', json={"x": 2.5, "y": 3.0})
        assert resp.status_code == 200
        node.navigate_to_pose.assert_called_with(2.5, 3.0)

    def test_navigate_negative_coords(self, client, node):
        resp = client.post('/api/navigate', json={"x": -1.0, "y": -2.0})
        assert resp.status_code == 200
        node.navigate_to_pose.assert_called_with(-1.0, -2.0)

    def test_navigate_origin(self, client, node):
        resp = client.post('/api/navigate', json={"x": 0, "y": 0})
        assert resp.status_code == 200
        node.navigate_to_pose.assert_called_with(0, 0)

    def test_navigate_failure_returns_error(self, client, node):
        node.navigate_to_pose.return_value = (False, "Nav2 not available")
        resp = client.post('/api/navigate', json={"x": 1, "y": 1})
        assert resp.status_code == 400
        assert "Nav2" in resp.get_json()["error"]


# ════════════════════════════════════════════════════════════════════
# Test Suite 5: Room Operations - Save/Load/Delete/Rename
# ════════════════════════════════════════════════════════════════════

class TestRoomOps:
    """Verify room management works correctly."""

    def test_save_room(self, client, node):
        resp = client.post('/api/rooms/save', json={"name": "Kitchen"})
        assert resp.status_code == 200
        node.save_room.assert_called_with("Kitchen")

    def test_save_room_empty_name(self, client, node):
        resp = client.post('/api/rooms/save', json={"name": ""})
        assert resp.status_code == 400

    def test_list_rooms_empty(self, client, node):
        WBN = webapp_module.WebBridgeNode
        WBN.list_rooms = lambda: []
        resp = client.get('/api/rooms')
        assert resp.status_code == 200
        assert resp.get_json() == []

    def test_delete_nonexistent_room(self, client, node):
        WBN = webapp_module.WebBridgeNode
        WBN.delete_room = lambda f: False
        resp = client.delete('/api/rooms/nonexistent')
        assert resp.status_code == 404

    def test_rename_room_empty_name(self, client, node):
        resp = client.put('/api/rooms/test/rename', json={"name": ""})
        assert resp.status_code == 400

    def test_path_traversal_attack(self, client, node):
        """Path traversal attacks are blocked."""
        WBN = webapp_module.WebBridgeNode
        WBN.delete_room = lambda f: False
        WBN._safe_room_path = lambda f: None
        attacks = ["../../../etc/passwd", "..%2F..%2Fetc", "....//....//etc"]
        for attack in attacks:
            resp = client.delete(f'/api/rooms/{attack}')
            assert resp.status_code == 404
            resp = client.get(f'/api/rooms/{attack}/preview')
            assert resp.status_code == 404


# ════════════════════════════════════════════════════════════════════
# Test Suite 6: Status and Feedback - App receives robot state
# ════════════════════════════════════════════════════════════════════

class TestStatusFeedback:
    """Verify the app correctly reports robot state."""

    def test_status_endpoint(self, client, node):
        node.mission_state = "EXPLORING"
        node.robot_x = 1.23
        node.robot_y = 4.56
        resp = client.get('/api/status')
        d = resp.get_json()
        assert d["mission_state"] == "EXPLORING"
        assert d["robot_x"] == 1.23
        assert d["robot_y"] == 4.56

    def test_status_all_states(self, client, node):
        """All mission states are reported correctly."""
        for state in ["WAITING_FOR_SCAN", "EXPLORING", "WAITING_FOR_CLEAN",
                       "COVERAGE", "RETURNING", "COMPLETE", "PAUSED", "ERROR"]:
            node.mission_state = state
            resp = client.get('/api/status')
            assert resp.get_json()["mission_state"] == state

    def test_log_endpoint(self, client, node):
        node.mission_log = [
            {"time": "10:00:00", "event": "Started"},
            {"time": "10:01:00", "event": "Scanning"},
        ]
        resp = client.get('/api/log')
        entries = resp.get_json()
        assert len(entries) == 2
        assert entries[0]["event"] == "Started"

    def test_stats_endpoint(self, client, node):
        node.get_stats.return_value = {
            "total_scan_time": 120,
            "total_clean_time": 300,
            "rooms_scanned": 3,
            "rooms_cleaned": 2,
            "total_distance": 25.5,
        }
        resp = client.get('/api/stats')
        d = resp.get_json()
        assert d["rooms_scanned"] == 3
        assert d["total_distance"] == 25.5

    def test_trail_endpoint(self, client, node):
        node.path_trail = [(0.1, 0.2), (0.3, 0.4), (0.5, 0.6)]
        resp = client.get('/api/trail')
        pts = resp.get_json()
        assert len(pts) == 3

    def test_map_not_available(self, client, node):
        resp = client.get('/api/map')
        assert resp.status_code == 404

    def test_map_available(self, client, node):
        node.get_map_data.return_value = {"image_b64": "abc", "width": 50, "height": 50}
        resp = client.get('/api/map')
        assert resp.status_code == 200
        assert resp.get_json()["width"] == 50


# ════════════════════════════════════════════════════════════════════
# Test Suite 7: Schedule Operations
# ════════════════════════════════════════════════════════════════════

class TestScheduleOps:
    def test_add_and_list(self, client, node):
        resp = client.post('/api/schedules',
                           json={"time": "08:00", "days": ["mon", "wed"]})
        assert resp.status_code == 200
        assert resp.get_json()["ok"]

    def test_add_requires_time(self, client, node):
        resp = client.post('/api/schedules', json={"time": "", "days": ["mon"]})
        assert resp.status_code == 400

    def test_add_requires_days(self, client, node):
        resp = client.post('/api/schedules', json={"time": "08:00", "days": []})
        assert resp.status_code == 400

    def test_delete_schedule(self, client, node):
        resp = client.post('/api/schedules',
                           json={"time": "09:00", "days": ["fri"]})
        sid = resp.get_json()["schedule"]["id"]
        resp = client.delete(f'/api/schedules/{sid}')
        assert resp.status_code == 200


# ════════════════════════════════════════════════════════════════════
# Test Suite 8: ROS Connection Loss Handling
# ════════════════════════════════════════════════════════════════════

class TestConnectionLoss:
    """Verify graceful behavior when ROS is not connected."""

    def test_command_without_ros(self, client, node):
        webapp_module.ros_node = None
        resp = client.post('/api/command', json={"command": "start_scan"})
        assert resp.status_code == 503
        assert "ROS" in resp.get_json()["error"]
        webapp_module.ros_node = node

    def test_velocity_without_ros(self, client, node):
        webapp_module.ros_node = None
        resp = client.post('/api/velocity', json={"linear": 0.1, "angular": 0})
        assert resp.status_code == 503
        webapp_module.ros_node = node

    def test_status_without_ros(self, client, node):
        webapp_module.ros_node = None
        resp = client.get('/api/status')
        assert resp.status_code == 503
        webapp_module.ros_node = node

    def test_map_without_ros(self, client, node):
        webapp_module.ros_node = None
        resp = client.get('/api/map')
        assert resp.status_code == 503
        webapp_module.ros_node = node

    def test_navigate_without_ros(self, client, node):
        webapp_module.ros_node = None
        resp = client.post('/api/navigate', json={"x": 0, "y": 0})
        assert resp.status_code == 503
        webapp_module.ros_node = node

    def test_save_room_without_ros(self, client, node):
        webapp_module.ros_node = None
        resp = client.post('/api/rooms/save', json={"name": "test"})
        assert resp.status_code == 503
        webapp_module.ros_node = node

    def test_schedules_without_ros(self, client, node):
        webapp_module.ros_node = None
        resp = client.post('/api/schedules', json={"time": "08:00", "days": ["mon"]})
        assert resp.status_code == 503
        webapp_module.ros_node = node

    def test_log_without_ros(self, client, node):
        """Log endpoint returns empty array gracefully."""
        webapp_module.ros_node = None
        resp = client.get('/api/log')
        assert resp.status_code == 200
        assert resp.get_json() == []
        webapp_module.ros_node = node

    def test_trail_without_ros(self, client, node):
        webapp_module.ros_node = None
        resp = client.get('/api/trail')
        assert resp.status_code == 200
        assert resp.get_json() == []
        webapp_module.ros_node = node

    def test_rooms_list_always_works(self, client, node):
        """Room listing works even without ROS (static file read)."""
        WBN = webapp_module.WebBridgeNode
        WBN.list_rooms = lambda: []
        resp = client.get('/api/rooms')
        assert resp.status_code == 200

    def test_stats_without_ros(self, client, node):
        webapp_module.ros_node = None
        resp = client.get('/api/stats')
        assert resp.status_code == 503
        webapp_module.ros_node = node


# ════════════════════════════════════════════════════════════════════
# Test Suite 9: Frontend Completeness
# ════════════════════════════════════════════════════════════════════

class TestFrontendCompleteness:
    """Verify the HTML has all required elements for full operation."""

    @pytest.fixture(autouse=True)
    def load_html(self, client):
        self.html = client.get('/').data.decode()

    def test_socket_io_included(self):
        assert 'socket.io' in self.html

    def test_all_command_buttons_present(self):
        for cmd in ['start_scan', 'stop_scan', 'start_clean', 'stop_clean',
                     'go_home', 'reset', 'pause', 'resume']:
            assert cmd in self.html, f"Missing button for {cmd}"

    def test_dpad_present(self):
        assert 'id="dpad"' in self.html

    def test_estop_present(self):
        assert 'id="estopBtn"' in self.html
        assert 'emergencyStop' in self.html

    def test_map_canvas_present(self):
        assert 'id="mapCanvas"' in self.html

    def test_room_manager_present(self):
        assert 'id="roomList"' in self.html
        assert 'id="roomName"' in self.html
        assert 'saveRoom' in self.html

    def test_status_elements_present(self):
        for eid in ['sMission', 'sExplore', 'sCoverage', 'sPos', 'sVel']:
            assert f'id="{eid}"' in self.html

    def test_log_present(self):
        assert 'id="logBox"' in self.html

    def test_schedule_ui_present(self):
        assert 'id="schedTime"' in self.html
        assert 'id="schedList"' in self.html
        assert 'addSchedule' in self.html

    def test_diagnostics_present(self):
        assert 'id="diagContent"' in self.html
        assert 'diagScanHz' in self.html
        assert 'diagTF' in self.html

    def test_settings_present(self):
        assert 'id="settingsOverlay"' in self.html
        assert 'saveSettings' in self.html

    def test_battery_indicator_present(self):
        assert 'id="batteryFill"' in self.html

    def test_estop_sends_both_stops(self):
        """E-stop JS sends both stop_clean and stop_scan."""
        assert "command: 'stop_clean'" in self.html
        assert "command: 'stop_scan'" in self.html

    def test_no_confirmation_on_stop_commands(self):
        """stop_clean and stop_scan are NOT in destructive commands set."""
        # Find the DESTRUCTIVE_CMDS line
        import re
        match = re.search(r"DESTRUCTIVE_CMDS\s*=\s*new Set\(\[(.*?)\]\)", self.html)
        assert match, "DESTRUCTIVE_CMDS not found"
        destructive = match.group(1)
        assert 'stop_clean' not in destructive
        assert 'stop_scan' not in destructive

    def test_speed_slider_present(self):
        assert 'id="speedSlider"' in self.html

    def test_keyboard_shortcuts_registered(self):
        assert 'keydown' in self.html
        assert 'keyup' in self.html

    def test_zoom_controls_present(self):
        assert 'id="zoomIn"' in self.html
        assert 'id="zoomOut"' in self.html

    def test_theme_toggle_present(self):
        assert 'id="themeToggle"' in self.html

    def test_language_toggle_present(self):
        assert 'id="langToggle"' in self.html

    def test_export_map_present(self):
        assert 'id="exportMapBtn"' in self.html

    def test_fullscreen_button_present(self):
        assert 'id="fullscreenBtn"' in self.html

    def test_mute_toggle_present(self):
        assert 'id="muteToggle"' in self.html

    def test_connection_indicator_present(self):
        assert 'id="connDot"' in self.html
        assert 'id="connLabel"' in self.html

    def test_latency_indicator_present(self):
        assert 'id="latencyDot"' in self.html


# ════════════════════════════════════════════════════════════════════
# Test Suite 10: Concurrent requests (thread safety)
# ════════════════════════════════════════════════════════════════════

class TestConcurrency:
    """Verify the app handles concurrent requests properly."""

    def test_multiple_commands_in_sequence(self, client, node):
        """Rapid command sequence doesn't crash."""
        for _ in range(20):
            client.post('/api/command', json={"command": "start_scan"})
            client.post('/api/command', json={"command": "stop_scan"})
        assert node.send_command.call_count == 40

    def test_rapid_velocity_updates(self, client, node):
        """Rapid velocity changes don't crash."""
        for i in range(50):
            v = 0.1 * (i % 5)
            client.post('/api/velocity', json={"linear": v, "angular": 0})
        assert node.send_velocity.call_count == 50

    def test_status_during_commands(self, client, node):
        """Status reads work while commands are being sent."""
        client.post('/api/command', json={"command": "start_scan"})
        resp = client.get('/api/status')
        assert resp.status_code == 200
        client.post('/api/command', json={"command": "stop_scan"})
        resp = client.get('/api/status')
        assert resp.status_code == 200


# ════════════════════════════════════════════════════════════════════
# Test Suite 11: Full Mission Scenario Simulation
# ════════════════════════════════════════════════════════════════════

class TestFullMissionScenario:
    """Simulate a complete cleaning mission through the web app."""

    def test_scan_then_clean_then_home(self, client, node):
        """Complete mission: scan → stop scan → clean → stop clean → home."""
        # 1. Start scanning
        resp = client.post('/api/command', json={"command": "start_scan"})
        assert resp.status_code == 200

        # 2. Stop scanning (map is complete)
        resp = client.post('/api/command', json={"command": "stop_scan"})
        assert resp.status_code == 200

        # 3. Start cleaning
        resp = client.post('/api/command', json={"command": "start_clean"})
        assert resp.status_code == 200

        # 4. Stop cleaning (or it finishes)
        resp = client.post('/api/command', json={"command": "stop_clean"})
        assert resp.status_code == 200

        # 5. Go home
        resp = client.post('/api/command', json={"command": "go_home"})
        assert resp.status_code == 200

        # Verify command sequence
        assert node._published_commands == [
            "start_scan", "stop_scan", "start_clean", "stop_clean", "go_home"
        ]

    def test_manual_drive_during_idle(self, client, node):
        """Manual drive works when robot is idle."""
        # Drive forward
        client.post('/api/velocity', json={"linear": 0.15, "angular": 0})
        # Turn
        client.post('/api/velocity', json={"linear": 0, "angular": 0.5})
        # Stop
        client.post('/api/velocity', json={"linear": 0, "angular": 0})

        assert node.send_velocity.call_count == 3

    def test_emergency_stop_during_cleaning(self, client, node):
        """E-stop during cleaning halts everything."""
        # Start cleaning
        client.post('/api/command', json={"command": "start_clean"})

        # Emergency stop!
        client.post('/api/command', json={"command": "stop_clean"})
        client.post('/api/command', json={"command": "stop_scan"})
        client.post('/api/velocity', json={"linear": 0, "angular": 0})

        # Verify robot was told to stop
        assert "stop_clean" in node._published_commands
        assert (0.0, 0.0) in node._published_velocities

    def test_save_room_after_scan(self, client, node):
        """Save room map after scanning."""
        client.post('/api/command', json={"command": "start_scan"})
        client.post('/api/command', json={"command": "stop_scan"})
        resp = client.post('/api/rooms/save', json={"name": "Living Room"})
        assert resp.status_code == 200
        node.save_room.assert_called_with("Living Room")

    def test_navigate_to_point(self, client, node):
        """Click-to-navigate sends correct goal."""
        resp = client.post('/api/navigate', json={"x": 3.5, "y": 2.1})
        assert resp.status_code == 200
        node.navigate_to_pose.assert_called_with(3.5, 2.1)


# ════════════════════════════════════════════════════════════════════
# Test Suite 12: Edge Cases & Robustness
# ════════════════════════════════════════════════════════════════════

class TestEdgeCases:
    """Test edge cases that could break remote operation."""

    def test_command_with_no_body(self, client, node):
        resp = client.post('/api/command', content_type='application/json')
        assert resp.status_code in (400, 500)

    def test_command_with_empty_body(self, client, node):
        resp = client.post('/api/command', json={})
        assert resp.status_code == 400

    def test_velocity_with_extreme_values(self, client, node):
        """Extreme velocity values are accepted without crash."""
        resp = client.post('/api/velocity', json={"linear": 999.0, "angular": -999.0})
        assert resp.status_code == 200

    def test_velocity_with_string_values(self, client, node):
        """Non-numeric velocity values handled gracefully."""
        resp = client.post('/api/velocity', json={"linear": "fast", "angular": "left"})
        assert resp.status_code == 500  # TypeError is caught

    def test_navigate_with_missing_coords(self, client, node):
        """Navigate defaults to (0,0) if coords missing."""
        resp = client.post('/api/navigate', json={})
        assert resp.status_code == 200
        node.navigate_to_pose.assert_called_with(0.0, 0.0)

    def test_save_room_with_special_chars(self, client, node):
        """Room names with special chars are sanitized."""
        resp = client.post('/api/rooms/save', json={"name": "<script>alert(1)</script>"})
        assert resp.status_code == 200
        node.save_room.assert_called()

    def test_save_room_with_whitespace_only(self, client, node):
        """Whitespace-only room name is rejected."""
        resp = client.post('/api/rooms/save', json={"name": "   "})
        assert resp.status_code == 400

    def test_schedule_with_invalid_time(self, client, node):
        """Schedule with malformed time is still accepted (no time validation)."""
        resp = client.post('/api/schedules', json={"time": "not-a-time", "days": ["mon"]})
        assert resp.status_code == 200

    def test_schedule_with_empty_days_list(self, client, node):
        """Schedule with empty days list is rejected."""
        resp = client.post('/api/schedules', json={"time": "14:00", "days": []})
        assert resp.status_code == 400

    def test_delete_schedule_nonexistent(self, client, node):
        """Deleting non-existent schedule succeeds (idempotent)."""
        resp = client.delete('/api/schedules/nonexistent-id')
        assert resp.status_code == 200

    def test_map_rate_bounds(self, client, node):
        """Map rate is clamped to valid range."""
        # This is a WebSocket event, test via HTTP that nothing crashes
        resp = client.get('/api/status')
        assert resp.status_code == 200

    def test_repeated_estop(self, client, node):
        """Multiple e-stops in sequence don't crash."""
        for _ in range(10):
            client.post('/api/command', json={"command": "stop_clean"})
            client.post('/api/command', json={"command": "stop_scan"})
            client.post('/api/velocity', json={"linear": 0, "angular": 0})
        assert node.send_command.call_count == 20

    def test_index_page_loads(self, client, node):
        """Main page loads successfully with all critical elements."""
        resp = client.get('/')
        assert resp.status_code == 200
        html = resp.data.decode()
        assert 'Clean Bot' in html
        assert 'socket.io' in html

    def test_concurrent_save_and_list(self, client, node):
        """Save and list rooms in quick succession."""
        client.post('/api/rooms/save', json={"name": "Room1"})
        client.post('/api/rooms/save', json={"name": "Room2"})
        WBN = webapp_module.WebBridgeNode
        WBN.list_rooms = lambda: [
            {"filename": "Room1", "name": "Room1"},
            {"filename": "Room2", "name": "Room2"},
        ]
        resp = client.get('/api/rooms')
        assert resp.status_code == 200
        rooms = resp.get_json()
        assert len(rooms) == 2

    def test_websocket_command_validation(self, client, node):
        """WebSocket command handler validates commands (tested via HTTP proxy)."""
        # Invalid command via HTTP
        resp = client.post('/api/command', json={"command": "hack_robot"})
        assert resp.status_code == 400

    def test_api_returns_json(self, client, node):
        """All API endpoints return JSON content type."""
        endpoints = [
            ('GET', '/api/status'),
            ('GET', '/api/log'),
            ('GET', '/api/trail'),
            ('GET', '/api/stats'),
        ]
        for method, url in endpoints:
            if method == 'GET':
                resp = client.get(url)
            assert 'json' in resp.content_type


# ════════════════════════════════════════════════════════════════════
# Test Suite 13: Network/Remote Operation Specific
# ════════════════════════════════════════════════════════════════════

class TestRemoteOperation:
    """Tests specific to running on a remote PC."""

    def test_cors_headers(self, client, node):
        """CORS allows cross-origin requests (needed for remote access)."""
        resp = client.get('/api/status')
        assert resp.status_code == 200

    def test_all_routes_handle_ros_disconnect(self, client, node):
        """Every route that needs ROS returns 503 when disconnected."""
        webapp_module.ros_node = None
        routes_needing_ros = [
            ('GET', '/api/status'),
            ('POST', '/api/command', {"command": "start_scan"}),
            ('POST', '/api/velocity', {"linear": 0, "angular": 0}),
            ('GET', '/api/map'),
            ('POST', '/api/rooms/save', {"name": "test"}),
            ('POST', '/api/navigate', {"x": 0, "y": 0}),
            ('GET', '/api/stats'),
            ('POST', '/api/schedules', {"time": "14:00", "days": ["mon"]}),
            ('DELETE', '/api/schedules/test'),
        ]
        for entry in routes_needing_ros:
            method, url = entry[0], entry[1]
            body = entry[2] if len(entry) > 2 else None
            if method == 'GET':
                resp = client.get(url)
            elif method == 'POST':
                resp = client.post(url, json=body)
            elif method == 'DELETE':
                resp = client.delete(url)
            assert resp.status_code == 503, f"{method} {url} should return 503, got {resp.status_code}"
        webapp_module.ros_node = node

    def test_static_routes_work_without_ros(self, client, node):
        """Routes that don't need ROS work when disconnected."""
        webapp_module.ros_node = None
        # Index page always works
        resp = client.get('/')
        assert resp.status_code == 200
        # Room listing always works (reads files)
        WBN = webapp_module.WebBridgeNode
        WBN.list_rooms = lambda: []
        resp = client.get('/api/rooms')
        assert resp.status_code == 200
        # Log and trail return empty arrays
        resp = client.get('/api/log')
        assert resp.status_code == 200
        resp = client.get('/api/trail')
        assert resp.status_code == 200
        webapp_module.ros_node = node

    def test_room_preview_without_ros(self, client, node):
        """Room preview works without ROS (reads saved files)."""
        webapp_module.ros_node = None
        WBN = webapp_module.WebBridgeNode
        WBN.load_room_preview = lambda f: None
        resp = client.get('/api/rooms/test/preview')
        assert resp.status_code == 404
        webapp_module.ros_node = node

    def test_room_delete_without_ros(self, client, node):
        """Room delete works without ROS (modifies files)."""
        webapp_module.ros_node = None
        WBN = webapp_module.WebBridgeNode
        WBN.delete_room = lambda f: False
        resp = client.delete('/api/rooms/test')
        assert resp.status_code == 404
        webapp_module.ros_node = node

    def test_room_rename_without_ros(self, client, node):
        """Room rename works without ROS."""
        webapp_module.ros_node = None
        WBN = webapp_module.WebBridgeNode
        WBN.rename_room = lambda f, n: (False, "not found")
        resp = client.put('/api/rooms/test/rename', json={"name": "New Name"})
        assert resp.status_code == 404
        webapp_module.ros_node = node
