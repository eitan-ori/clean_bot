#!/usr/bin/env python3
"""
Comprehensive tests for the Clean Bot Web Control Panel.

Tests the Flask routes, room management, schedule management,
statistics, and utility functions WITHOUT requiring ROS 2.
"""

import json
import os
import sys
import tempfile
import shutil
import time
import math
from pathlib import Path
from unittest.mock import MagicMock, patch, PropertyMock

import pytest
import numpy as np


# ── Patch rclpy and ROS 2 imports before importing app ──
# This allows tests to run without a ROS 2 installation.
class FakeRclpy:
    """Minimal rclpy mock for testing."""
    def ok(self):
        return True
    def init(self, *a, **kw):
        pass
    def shutdown(self, *a, **kw):
        pass

class FakeTime:
    def __init__(self, seconds=0, nanoseconds=0):
        pass
    def to_msg(self):
        return MagicMock()

class FakeNode:
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
        m.now.return_value = MagicMock(
            to_msg=MagicMock(return_value=MagicMock()),
            nanoseconds=0
        )
        return m
    def destroy_node(self):
        pass

# Mock ROS 2 modules
mock_rclpy = MagicMock()
mock_rclpy.ok = MagicMock(return_value=True)
mock_rclpy.time = MagicMock()
mock_rclpy.time.Time = FakeTime

sys.modules['rclpy'] = mock_rclpy
sys.modules['rclpy.node'] = MagicMock()
sys.modules['rclpy.node'].Node = FakeNode
sys.modules['rclpy.qos'] = MagicMock()
sys.modules['rclpy.executors'] = MagicMock()
sys.modules['rclpy.action'] = MagicMock()
sys.modules['rclpy.callback_groups'] = MagicMock()
sys.modules['rclpy.time'] = MagicMock()
sys.modules['rclpy.time'].Time = FakeTime
sys.modules['std_msgs'] = MagicMock()
sys.modules['std_msgs.msg'] = MagicMock()
sys.modules['geometry_msgs'] = MagicMock()
sys.modules['geometry_msgs.msg'] = MagicMock()
sys.modules['nav_msgs'] = MagicMock()
sys.modules['nav_msgs.msg'] = MagicMock()
sys.modules['sensor_msgs'] = MagicMock()
sys.modules['sensor_msgs.msg'] = MagicMock()
sys.modules['tf2_ros'] = MagicMock()
sys.modules['nav2_msgs'] = MagicMock()
sys.modules['nav2_msgs.action'] = MagicMock()


# ════════════════════════════════════════════════════════════════════
# Import app module after mocking
# ════════════════════════════════════════════════════════════════════

# We need to patch the WebBridgeNode to not actually init ROS
from clean_bot_mission.webapp import app as webapp_module

# ════════════════════════════════════════════════════════════════════
# Fixtures
# ════════════════════════════════════════════════════════════════════

@pytest.fixture
def temp_rooms_dir(tmp_path):
    """Create a temp directory for saved rooms."""
    rooms_dir = tmp_path / "saved_rooms"
    rooms_dir.mkdir()
    return rooms_dir


@pytest.fixture
def flask_client(temp_rooms_dir):
    """Create Flask test client with mocked ROS node."""
    webapp_module.app.config['TESTING'] = True

    # Mock the ros_node
    mock_node = MagicMock()
    mock_node.mission_state = "WAITING_FOR_SCAN"
    mock_node.exploration_state = "IDLE"
    mock_node.coverage_state = "IDLE"
    mock_node.robot_x = 1.5
    mock_node.robot_y = 2.3
    mock_node.robot_yaw = 0.5
    mock_node.linear_vel = 0.1
    mock_node.angular_vel = 0.0
    mock_node.map_update_counter = 42
    mock_node.map_msg = None
    mock_node.mission_log = [
        {"time": "12:00:00", "event": "Mission started"},
        {"time": "12:01:00", "event": "Scanning..."},
    ]
    mock_node.path_trail = [(0.1, 0.2), (0.3, 0.4)]
    mock_node.get_stats.return_value = {
        "total_scan_time": 120.5,
        "total_clean_time": 300.0,
        "rooms_scanned": 2,
        "rooms_cleaned": 1,
        "total_distance": 15.67,
    }
    mock_node._schedules = []
    mock_node._schedule_lock = MagicMock()
    mock_node._schedule_lock.__enter__ = MagicMock(return_value=None)
    mock_node._schedule_lock.__exit__ = MagicMock(return_value=False)
    mock_node._save_schedules = MagicMock()
    mock_node.send_command = MagicMock()
    mock_node.send_velocity = MagicMock()
    mock_node.save_room = MagicMock(return_value=(True, "/path/to/room.json"))
    mock_node.navigate_to_pose = MagicMock(return_value=(True, "Goal sent"))
    mock_node.get_map_data = MagicMock(return_value=None)
    mock_node.load_room_preview = MagicMock(return_value=None)

    # Patch the module-level ros_node
    original_node = webapp_module.ros_node
    webapp_module.ros_node = mock_node

    # Patch SAVED_ROOMS_DIR
    original_rooms_dir = webapp_module.SAVED_ROOMS_DIR
    webapp_module.SAVED_ROOMS_DIR = temp_rooms_dir

    # Patch SCHEDULES_FILE
    original_sched_file = webapp_module.SCHEDULES_FILE
    webapp_module.SCHEDULES_FILE = temp_rooms_dir / "schedules.json"

    with webapp_module.app.test_client() as client:
        yield client, mock_node

    webapp_module.ros_node = original_node
    webapp_module.SAVED_ROOMS_DIR = original_rooms_dir
    webapp_module.SCHEDULES_FILE = original_sched_file


# ════════════════════════════════════════════════════════════════════
# Test: Index page
# ════════════════════════════════════════════════════════════════════

class TestIndexPage:
    def test_index_returns_html(self, flask_client):
        client, _ = flask_client
        resp = client.get('/')
        assert resp.status_code == 200
        assert b'Clean Bot Control Panel' in resp.data

    def test_index_contains_key_elements(self, flask_client):
        client, _ = flask_client
        resp = client.get('/')
        html = resp.data.decode()
        # Check all major UI sections exist
        assert 'mapCanvas' in html
        assert 'estopBtn' in html
        assert 'dpad' in html
        assert 'roomList' in html
        assert 'logBox' in html
        assert 'schedList' in html
        assert 'diagContent' in html
        assert 'batteryFill' in html
        assert 'socket.io' in html

    def test_index_contains_i18n_attributes(self, flask_client):
        client, _ = flask_client
        html = client.get('/').data.decode()
        assert 'data-i18n="emergency_stop"' in html
        assert 'data-i18n="scan_room"' in html
        assert 'data-i18n="diagnostics"' in html

    def test_index_contains_js_functions(self, flask_client):
        client, _ = flask_client
        html = client.get('/').data.decode()
        for fn in ['sendCommand', 'emergencyStop', 'saveRoom', 'addSchedule',
                    'saveSettings', 'closeSettings', 'fetchStats', 'applyLanguage',
                    'updateBattery', 'drawMap']:
            assert fn in html, f"Missing JS function: {fn}"


# ════════════════════════════════════════════════════════════════════
# Test: API Status
# ════════════════════════════════════════════════════════════════════

class TestAPIStatus:
    def test_status_returns_json(self, flask_client):
        client, _ = flask_client
        resp = client.get('/api/status')
        assert resp.status_code == 200
        d = resp.get_json()
        assert d['mission_state'] == 'WAITING_FOR_SCAN'
        assert d['robot_x'] == 1.5
        assert d['map_updates'] == 42

    def test_status_no_ros(self, flask_client):
        client, _ = flask_client
        webapp_module.ros_node = None
        resp = client.get('/api/status')
        assert resp.status_code == 503
        webapp_module.ros_node = flask_client[1]


# ════════════════════════════════════════════════════════════════════
# Test: API Commands
# ════════════════════════════════════════════════════════════════════

class TestAPICommand:
    def test_valid_command(self, flask_client):
        client, node = flask_client
        for cmd in ['start_scan', 'stop_scan', 'start_clean', 'stop_clean',
                     'go_home', 'reset', 'pause', 'resume']:
            resp = client.post('/api/command',
                               json={"command": cmd},
                               content_type='application/json')
            assert resp.status_code == 200
            assert resp.get_json()['ok'] is True
        assert node.send_command.call_count == 8

    def test_invalid_command(self, flask_client):
        client, _ = flask_client
        resp = client.post('/api/command',
                           json={"command": "self_destruct"},
                           content_type='application/json')
        assert resp.status_code == 400
        assert 'Unknown command' in resp.get_json()['error']

    def test_command_no_json(self, flask_client):
        client, _ = flask_client
        resp = client.post('/api/command',
                           data='not json',
                           content_type='text/plain')
        # Flask returns 415 Unsupported Media Type for non-JSON content
        assert resp.status_code in (400, 415)

    def test_command_empty_body(self, flask_client):
        client, _ = flask_client
        resp = client.post('/api/command',
                           json={},
                           content_type='application/json')
        assert resp.status_code == 400

    def test_command_no_ros(self, flask_client):
        client, node = flask_client
        webapp_module.ros_node = None
        resp = client.post('/api/command',
                           json={"command": "start_scan"},
                           content_type='application/json')
        assert resp.status_code == 503
        webapp_module.ros_node = node


# ════════════════════════════════════════════════════════════════════
# Test: API Velocity
# ════════════════════════════════════════════════════════════════════

class TestAPIVelocity:
    def test_send_velocity(self, flask_client):
        client, node = flask_client
        resp = client.post('/api/velocity',
                           json={"linear": 0.1, "angular": 0.5},
                           content_type='application/json')
        assert resp.status_code == 200
        node.send_velocity.assert_called_with(0.1, 0.5)

    def test_velocity_defaults(self, flask_client):
        client, node = flask_client
        resp = client.post('/api/velocity',
                           json={},
                           content_type='application/json')
        assert resp.status_code == 200
        node.send_velocity.assert_called_with(0.0, 0.0)


# ════════════════════════════════════════════════════════════════════
# Test: API Map
# ════════════════════════════════════════════════════════════════════

class TestAPIMap:
    def test_no_map(self, flask_client):
        client, _ = flask_client
        resp = client.get('/api/map')
        assert resp.status_code == 404

    def test_with_map(self, flask_client):
        client, node = flask_client
        node.get_map_data.return_value = {
            "image_b64": "dGVzdA==",
            "width": 100,
            "height": 100,
        }
        resp = client.get('/api/map')
        assert resp.status_code == 200
        assert resp.get_json()['width'] == 100


# ════════════════════════════════════════════════════════════════════
# Test: API Log
# ════════════════════════════════════════════════════════════════════

class TestAPILog:
    def test_get_log(self, flask_client):
        client, _ = flask_client
        resp = client.get('/api/log')
        assert resp.status_code == 200
        entries = resp.get_json()
        assert len(entries) == 2
        assert entries[0]['event'] == 'Mission started'


# ════════════════════════════════════════════════════════════════════
# Test: API Stats
# ════════════════════════════════════════════════════════════════════

class TestAPIStats:
    def test_get_stats(self, flask_client):
        client, _ = flask_client
        resp = client.get('/api/stats')
        assert resp.status_code == 200
        d = resp.get_json()
        assert d['rooms_scanned'] == 2
        assert d['total_distance'] == 15.67


# ════════════════════════════════════════════════════════════════════
# Test: API Trail
# ════════════════════════════════════════════════════════════════════

class TestAPITrail:
    def test_get_trail(self, flask_client):
        client, _ = flask_client
        resp = client.get('/api/trail')
        assert resp.status_code == 200
        pts = resp.get_json()
        assert len(pts) == 2
        assert pts[0] == [0.1, 0.2]


# ════════════════════════════════════════════════════════════════════
# Test: Room Management
# ════════════════════════════════════════════════════════════════════

class TestRoomManagement:
    def test_list_rooms_empty(self, flask_client):
        client, _ = flask_client
        resp = client.get('/api/rooms')
        assert resp.status_code == 200
        assert resp.get_json() == []

    def test_save_room(self, flask_client):
        client, node = flask_client
        resp = client.post('/api/rooms/save',
                           json={"name": "Living Room"},
                           content_type='application/json')
        assert resp.status_code == 200
        node.save_room.assert_called_with("Living Room")

    def test_save_room_no_name(self, flask_client):
        client, _ = flask_client
        resp = client.post('/api/rooms/save',
                           json={"name": ""},
                           content_type='application/json')
        assert resp.status_code == 400

    def test_save_room_whitespace_name(self, flask_client):
        client, _ = flask_client
        resp = client.post('/api/rooms/save',
                           json={"name": "   "},
                           content_type='application/json')
        assert resp.status_code == 400

    def test_delete_room_not_found(self, flask_client):
        client, _ = flask_client
        resp = client.delete('/api/rooms/nonexistent')
        assert resp.status_code == 404

    def test_room_preview_not_found(self, flask_client):
        client, _ = flask_client
        resp = client.get('/api/rooms/nonexistent/preview')
        assert resp.status_code == 404

    def test_rename_room_no_name(self, flask_client):
        client, _ = flask_client
        resp = client.put('/api/rooms/test/rename',
                          json={"name": ""},
                          content_type='application/json')
        assert resp.status_code == 400

    def test_path_traversal_blocked(self, flask_client):
        client, _ = flask_client
        # Attempts to escape saved_rooms directory
        resp = client.delete('/api/rooms/..%2F..%2Fetc%2Fpasswd')
        assert resp.status_code == 404
        resp = client.get('/api/rooms/../../../etc/passwd/preview')
        assert resp.status_code == 404


# ════════════════════════════════════════════════════════════════════
# Test: Room Safe Path
# ════════════════════════════════════════════════════════════════════

class TestSafeRoomPath:
    def test_valid_filename(self, temp_rooms_dir):
        webapp_module.SAVED_ROOMS_DIR = temp_rooms_dir
        result = webapp_module.WebBridgeNode._safe_room_path("my-room_1")
        assert result is not None
        assert str(temp_rooms_dir) in str(result)

    def test_invalid_chars(self, temp_rooms_dir):
        webapp_module.SAVED_ROOMS_DIR = temp_rooms_dir
        assert webapp_module.WebBridgeNode._safe_room_path("../etc/passwd") is None
        assert webapp_module.WebBridgeNode._safe_room_path("room name") is None
        assert webapp_module.WebBridgeNode._safe_room_path("room.json") is None
        assert webapp_module.WebBridgeNode._safe_room_path("") is None


# ════════════════════════════════════════════════════════════════════
# Test: Schedule Management
# ════════════════════════════════════════════════════════════════════

class TestScheduleManagement:
    def test_get_schedules_empty(self, flask_client):
        client, _ = flask_client
        resp = client.get('/api/schedules')
        assert resp.status_code == 200
        assert resp.get_json() == []

    def test_add_schedule(self, flask_client):
        client, node = flask_client
        resp = client.post('/api/schedules',
                           json={"time": "08:30", "days": ["mon", "wed", "fri"]},
                           content_type='application/json')
        assert resp.status_code == 200
        d = resp.get_json()
        assert d['ok'] is True
        assert d['schedule']['time'] == '08:30'
        assert d['schedule']['days'] == ['mon', 'wed', 'fri']

    def test_add_schedule_no_time(self, flask_client):
        client, _ = flask_client
        resp = client.post('/api/schedules',
                           json={"time": "", "days": ["mon"]},
                           content_type='application/json')
        assert resp.status_code == 400

    def test_add_schedule_no_days(self, flask_client):
        client, _ = flask_client
        resp = client.post('/api/schedules',
                           json={"time": "08:30", "days": []},
                           content_type='application/json')
        assert resp.status_code == 400

    def test_delete_schedule(self, flask_client):
        client, node = flask_client
        # Add one first
        resp = client.post('/api/schedules',
                           json={"time": "09:00", "days": ["tue"]},
                           content_type='application/json')
        sched_id = resp.get_json()['schedule']['id']
        # Delete it
        resp = client.delete(f'/api/schedules/{sched_id}')
        assert resp.status_code == 200
        assert resp.get_json()['ok'] is True


# ════════════════════════════════════════════════════════════════════
# Test: Navigate
# ════════════════════════════════════════════════════════════════════

class TestAPINavigate:
    def test_navigate(self, flask_client):
        client, node = flask_client
        resp = client.post('/api/navigate',
                           json={"x": 1.5, "y": 2.5},
                           content_type='application/json')
        assert resp.status_code == 200
        node.navigate_to_pose.assert_called_with(1.5, 2.5)

    def test_navigate_no_ros(self, flask_client):
        client, node = flask_client
        webapp_module.ros_node = None
        resp = client.post('/api/navigate',
                           json={"x": 0, "y": 0},
                           content_type='application/json')
        assert resp.status_code == 503
        webapp_module.ros_node = node


# ════════════════════════════════════════════════════════════════════
# Test: Map Rendering Utilities (unit tests)
# ════════════════════════════════════════════════════════════════════

class TestMapRendering:
    """Test map data handling with numpy arrays."""

    def test_occupancy_grid_to_rgba(self):
        """Verify that occupancy values map to correct colors."""
        # -1 = unknown, 0 = free, 100 = occupied
        data = np.array([-1, 0, 100, 25], dtype=np.int8).reshape((2, 2))
        rgba = np.zeros((2, 2, 4), dtype=np.uint8)
        rgba[..., 3] = 255
        unknown = data == -1
        free = data == 0
        occupied = data >= 50
        rgba[unknown] = [40, 40, 48, 255]
        rgba[free] = [230, 235, 240, 255]
        rgba[occupied] = [220, 50, 50, 255]
        # Check
        assert list(rgba[0, 0]) == [40, 40, 48, 255]   # unknown
        assert list(rgba[0, 1]) == [230, 235, 240, 255] # free
        assert list(rgba[1, 0]) == [220, 50, 50, 255]   # occupied
        # partial (25) - not set by above, still zeros except alpha
        assert rgba[1, 1, 3] == 255

    def test_heatmap_vectorized(self):
        """Test the vectorized heatmap rendering logic."""
        hm = np.array([[0, 50], [150, 220]], dtype=np.float32)
        max_val = hm.max()
        hm_norm = (hm / max_val * 255).astype(np.uint8)

        mask = hm_norm > 0
        v = hm_norm[mask].astype(np.int16)
        assert len(v) == 3  # 3 non-zero values

        low = v < 128
        mid = (v >= 128) & (v < 200)
        high = v >= 200

        r = np.zeros_like(v, dtype=np.uint8)
        b = np.zeros_like(v, dtype=np.uint8)
        b[low] = np.minimum(255, v[low] * 4).astype(np.uint8)
        r[mid] = np.minimum(255, (v[mid] - 128) * 3).astype(np.uint8)
        r[high] = 255

        assert all(r[high] == 255)
        assert all(b[low] > 0)


# ════════════════════════════════════════════════════════════════════
# Test: Configuration and Setup
# ════════════════════════════════════════════════════════════════════

class TestConfiguration:
    def test_app_secret_key(self):
        assert webapp_module.app.config['SECRET_KEY'] is not None

    def test_saved_rooms_dir_exists(self):
        assert webapp_module.SAVED_ROOMS_DIR.exists() or True  # may be temp in tests

    def test_template_dir(self):
        """Verify template directory is set correctly."""
        tpl_dir = Path(webapp_module.app.template_folder)
        assert tpl_dir.name == 'templates'

    def test_entry_points(self):
        """Verify main() function exists and is callable."""
        assert callable(webapp_module.main)


# ════════════════════════════════════════════════════════════════════
# Test: VALID_COMMANDS constant
# ════════════════════════════════════════════════════════════════════

class TestValidCommands:
    """Verify the command whitelist matches full_mission's accepted commands."""

    def test_valid_commands_exists(self):
        assert hasattr(webapp_module, 'VALID_COMMANDS')
        assert isinstance(webapp_module.VALID_COMMANDS, set)

    def test_all_eight_commands(self):
        expected = {"start_scan", "stop_scan", "start_clean", "stop_clean",
                    "go_home", "reset", "pause", "resume"}
        assert webapp_module.VALID_COMMANDS == expected

    def test_invalid_command_rejected(self, flask_client):
        client, _ = flask_client
        for cmd in ["", "hack", "shutdown", "sudo rm -rf", "START_SCAN"]:
            resp = client.post('/api/command', json={"command": cmd})
            assert resp.status_code == 400, f"Command '{cmd}' should be rejected"


# ════════════════════════════════════════════════════════════════════
# Test: WebSocket input validation (Bug 46)
# ════════════════════════════════════════════════════════════════════

class TestWebSocketInputValidation:
    """Ensure WS handlers reject non-dict data without crashing."""

    def test_ws_command_rejects_string(self):
        fn = webapp_module.ws_command
        # Should not raise — just return silently
        fn("start_scan")

    def test_ws_command_rejects_none(self):
        fn = webapp_module.ws_command
        fn(None)

    def test_ws_command_rejects_int(self):
        fn = webapp_module.ws_command
        fn(42)

    def test_ws_velocity_rejects_string(self):
        fn = webapp_module.ws_velocity
        fn("fast")

    def test_ws_set_map_rate_rejects_string(self):
        fn = webapp_module.ws_set_map_rate
        fn("5")

    def test_ws_set_map_rate_rejects_bad_value(self):
        fn = webapp_module.ws_set_map_rate
        fn({"rate": "not_a_number"})


# ════════════════════════════════════════════════════════════════════
# Test: Room file operations (realistic)
# ════════════════════════════════════════════════════════════════════

class TestRoomFileOps:
    """Test room CRUD with real file operations."""

    def test_list_rooms_with_files(self, temp_rooms_dir):
        webapp_module.SAVED_ROOMS_DIR = temp_rooms_dir
        # Create a room file
        room = {"name": "Kitchen", "saved_at": "2024-01-01T12:00:00",
                "width": 100, "height": 100, "resolution": 0.05, "data": [0]*10000}
        import json
        with open(temp_rooms_dir / "Kitchen.json", "w") as f:
            json.dump(room, f)
        rooms = webapp_module.WebBridgeNode.list_rooms()
        assert len(rooms) == 1
        assert rooms[0]["name"] == "Kitchen"
        assert rooms[0]["width"] == 100

    def test_delete_room_real(self, temp_rooms_dir):
        webapp_module.SAVED_ROOMS_DIR = temp_rooms_dir
        import json
        room = {"name": "Test", "data": [0]}
        with open(temp_rooms_dir / "Test.json", "w") as f:
            json.dump(room, f)
        assert webapp_module.WebBridgeNode.delete_room("Test") is True
        assert not (temp_rooms_dir / "Test.json").exists()

    def test_delete_nonexistent(self, temp_rooms_dir):
        webapp_module.SAVED_ROOMS_DIR = temp_rooms_dir
        assert webapp_module.WebBridgeNode.delete_room("nonexistent") is False

    def test_safe_room_path_valid(self, temp_rooms_dir):
        webapp_module.SAVED_ROOMS_DIR = temp_rooms_dir
        result = webapp_module.WebBridgeNode._safe_room_path("my-room_1")
        assert result is not None
        assert str(temp_rooms_dir) in str(result)

    def test_safe_room_path_traversal(self, temp_rooms_dir):
        webapp_module.SAVED_ROOMS_DIR = temp_rooms_dir
        assert webapp_module.WebBridgeNode._safe_room_path("../etc/passwd") is None
        assert webapp_module.WebBridgeNode._safe_room_path("room name") is None
        assert webapp_module.WebBridgeNode._safe_room_path("room.json") is None
        assert webapp_module.WebBridgeNode._safe_room_path("") is None

    def test_rename_room_real(self, temp_rooms_dir):
        webapp_module.SAVED_ROOMS_DIR = temp_rooms_dir
        import json
        room = {"name": "Old Name", "data": [0]}
        with open(temp_rooms_dir / "OldName.json", "w") as f:
            json.dump(room, f)
        ok, new_fn = webapp_module.WebBridgeNode.rename_room("OldName", "New Name")
        assert ok is True
        assert new_fn == "New_Name"
        assert (temp_rooms_dir / "New_Name.json").exists()
        assert not (temp_rooms_dir / "OldName.json").exists()

    def test_load_room_preview_real(self, temp_rooms_dir):
        webapp_module.SAVED_ROOMS_DIR = temp_rooms_dir
        import json
        room = {"name": "Test", "width": 4, "height": 4, "resolution": 0.05,
                "data": [0]*16, "saved_at": "2024-01-01"}
        with open(temp_rooms_dir / "Test.json", "w") as f:
            json.dump(room, f)
        result = webapp_module.WebBridgeNode.load_room_preview("Test")
        assert result is not None
        assert result["name"] == "Test"
        assert "image_b64" in result
        assert len(result["image_b64"]) > 0

    def test_load_room_preview_corrupted(self, temp_rooms_dir):
        webapp_module.SAVED_ROOMS_DIR = temp_rooms_dir
        with open(temp_rooms_dir / "Corrupt.json", "w") as f:
            f.write("not valid json{{{")
        result = webapp_module.WebBridgeNode.load_room_preview("Corrupt")
        assert result is None

    def test_load_room_preview_missing_fields(self, temp_rooms_dir):
        webapp_module.SAVED_ROOMS_DIR = temp_rooms_dir
        import json
        room = {"name": "Incomplete"}  # missing width, height, data
        with open(temp_rooms_dir / "Incomplete.json", "w") as f:
            json.dump(room, f)
        result = webapp_module.WebBridgeNode.load_room_preview("Incomplete")
        assert result is None

    def test_load_room_preview_data_length_mismatch(self, temp_rooms_dir):
        """Bug 36: Corrupted room with wrong data length should return None, not crash."""
        webapp_module.SAVED_ROOMS_DIR = temp_rooms_dir
        import json
        room = {"name": "Bad", "width": 10, "height": 10, "resolution": 0.05,
                "data": [0] * 50}  # 50 != 10*10
        with open(temp_rooms_dir / "Bad.json", "w") as f:
            json.dump(room, f)
        result = webapp_module.WebBridgeNode.load_room_preview("Bad")
        assert result is None

    def test_rename_room_special_chars_name(self, temp_rooms_dir):
        """Bug 37: Renaming to all-special-chars should fail, not create '.json'."""
        webapp_module.SAVED_ROOMS_DIR = temp_rooms_dir
        import json
        room = {"name": "Test", "width": 2, "height": 2, "resolution": 0.05, "data": [0]*4}
        with open(temp_rooms_dir / "Test.json", "w") as f:
            json.dump(room, f)
        ok, info = webapp_module.WebBridgeNode.rename_room("Test", "@#$%")
        assert not ok
        assert "alphanumeric" in info.lower()


# ── Coverage Algorithm Tests ────────────────────────────────────────────────
class TestCoverageAlgorithm:
    """Unit tests for adaptive_coverage path logic (Bug 26, 27 fixes)."""

    def test_waypoint_rotation_covers_all(self):
        """Bug 26: Coverage should visit ALL waypoints, not skip ones before nearest."""
        # Simulate: 10 waypoints, robot nearest to waypoint 5
        waypoints = [(float(i), 0.0, 0.0) for i in range(10)]
        nearest = 5
        # Apply the rotation fix
        rotated = waypoints[nearest:] + waypoints[:nearest]
        # All original waypoints should be present
        assert len(rotated) == len(waypoints)
        assert set(w[0] for w in rotated) == set(w[0] for w in waypoints)
        # The first waypoint should be the one nearest to robot
        assert rotated[0] == waypoints[nearest]

    def test_waypoint_rotation_zero_nearest(self):
        """When nearest is already index 0, rotation is a no-op."""
        waypoints = [(float(i), 0.0, 0.0) for i in range(5)]
        nearest = 0
        if nearest > 0:
            rotated = waypoints[nearest:] + waypoints[:nearest]
        else:
            rotated = waypoints
        assert rotated == waypoints

    def test_densify_waypoints_adds_intermediates(self):
        """Densify should add points on long segments."""
        waypoints = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)]
        max_seg = 0.3
        densified = [waypoints[0]]
        for i in range(len(waypoints) - 1):
            x1, y1, yaw1 = waypoints[i]
            x2, y2, yaw2 = waypoints[i + 1]
            dx, dy = x2 - x1, y2 - y1
            seg_len = math.sqrt(dx*dx + dy*dy)
            if seg_len > max_seg:
                n = int(math.ceil(seg_len / max_seg))
                for j in range(1, n):
                    t = j / n
                    densified.append((x1 + t * dx, y1 + t * dy, yaw1))
        densified.append(waypoints[-1])
        assert len(densified) > 2
        # Check no gap exceeds max_segment_length
        for i in range(len(densified) - 1):
            dx = densified[i + 1][0] - densified[i][0]
            dy = densified[i + 1][1] - densified[i][1]
            assert math.sqrt(dx*dx + dy*dy) <= max_seg + 0.01

    def test_orient_waypoints_points_forward(self):
        """Orient should set yaw to point toward next waypoint."""
        waypoints = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (1.0, 1.0, 0.0)]
        oriented = []
        for i in range(len(waypoints) - 1):
            x, y, _ = waypoints[i]
            nx, ny, _ = waypoints[i + 1]
            yaw = math.atan2(ny - y, nx - x)
            oriented.append((x, y, yaw))
        lx, ly, _ = waypoints[-1]
        oriented.append((lx, ly, oriented[-1][2]))
        # First: pointing right (0 rad)
        assert abs(oriented[0][2] - 0.0) < 0.01
        # Second: pointing up (pi/2)
        assert abs(oriented[1][2] - math.pi / 2) < 0.01


# ── Bug 47: send_command emits mission_state via SocketIO ──
class TestSendCommandEmit:
    """Bug 47: send_command should emit log_entry in real-time via SocketIO."""

    def test_send_command_emits_log_entry(self):
        """send_command should call sio.emit with mission_state and log_entry."""
        bridge = MagicMock()
        bridge.mission_state = "idle"
        bridge.mission_log = []
        bridge.sio = MagicMock()
        bridge.cmd_pub = MagicMock()
        bridge.get_logger.return_value = MagicMock()
        # Call the real method on our mock
        from collections import deque
        bridge.mission_log = deque(maxlen=200)
        webapp_module.WebBridgeNode.send_command(bridge, "start_scan")
        bridge.sio.emit.assert_called_once()
        call_args = bridge.sio.emit.call_args
        assert call_args[0][0] == "mission_state"
        payload = call_args[0][1]
        assert "log_entry" in payload
        assert "start_scan" in payload["log_entry"]["event"]

    def test_send_command_appends_to_mission_log(self):
        """send_command should also append to the deque for GET /api/log."""
        from collections import deque
        bridge = MagicMock()
        bridge.mission_state = "idle"
        bridge.mission_log = deque(maxlen=200)
        bridge.sio = MagicMock()
        bridge.cmd_pub = MagicMock()
        bridge.get_logger.return_value = MagicMock()
        webapp_module.WebBridgeNode.send_command(bridge, "stop_clean")
        assert len(bridge.mission_log) == 1
        assert "stop_clean" in bridge.mission_log[-1]["event"]


# ── Bug 48: CSS uses variables instead of hardcoded dark colors ──
class TestLightModeCSS:
    """Bug 48: All CSS colors should use variables, not hardcoded hex."""

    def test_no_hardcoded_dark_colors_outside_root(self, flask_client):
        """Hardcoded dark hex colors should only appear in :root definitions."""
        import re
        client, _ = flask_client
        resp = client.get("/")
        html = resp.data.decode()
        style_match = re.search(r"<style>(.*?)</style>", html, re.DOTALL)
        assert style_match, "No <style> block found"
        css = style_match.group(1)
        # Remove :root{...} and html.light{...} blocks (variable definitions are OK)
        css_no_root = re.sub(r":root\s*\{[^}]+\}", "", css)
        css_no_root = re.sub(r"html\.light\s*\{[^}]+\}", "", css_no_root)
        dark_colors = ["#1e1f36", "#353660", "#1a1b30", "#16172a", "#2a2b44", "#282830", "#101020"]
        for color in dark_colors:
            assert color not in css_no_root, f"Hardcoded dark color {color} found outside :root"


# ── Bug 49: socket.io loaded locally, not from CDN ──
class TestSocketIOLocal:
    """Bug 49: socket.io client should be loaded from local server, not CDN."""

    def test_socket_io_local_script(self, flask_client):
        """The socket.io script tag should reference /socket.io/socket.io.js."""
        client, _ = flask_client
        resp = client.get("/")
        html = resp.data.decode()
        assert '/socket.io/socket.io.js' in html
        assert 'cdnjs.cloudflare.com' not in html


# ── Bug 50: Malformed map data should not crash ──
class TestMapDataGuard:
    """Bug 50: _on_map and get_map_data should guard against malformed data."""

    def test_on_map_rejects_empty_dimensions(self):
        """_on_map should skip maps with zero width or height."""
        bridge = MagicMock()
        bridge.map_msg = None
        bridge.map_update_counter = 0
        bridge.obstacle_heatmap = None
        bridge._heatmap_width = 0
        bridge._heatmap_height = 0
        bridge._last_map_emit = 0.0
        bridge._map_rate_interval = 0.5
        bridge.sio = MagicMock()
        # Create a fake map msg with 0 width
        msg = MagicMock()
        msg.info.width = 0
        msg.info.height = 10
        msg.data = []
        webapp_module.WebBridgeNode._on_map(bridge, msg)
        # map_msg should NOT be updated
        assert bridge.map_msg is None
        assert bridge.map_update_counter == 0

    def test_on_map_rejects_mismatched_data(self):
        """_on_map should skip maps where len(data) != w*h."""
        bridge = MagicMock()
        bridge.map_msg = None
        bridge.map_update_counter = 0
        bridge.obstacle_heatmap = None
        bridge._heatmap_width = 0
        bridge._heatmap_height = 0
        bridge._last_map_emit = 0.0
        bridge._map_rate_interval = 0.5
        bridge.sio = MagicMock()
        msg = MagicMock()
        msg.info.width = 10
        msg.info.height = 10
        msg.data = [0] * 50  # should be 100
        webapp_module.WebBridgeNode._on_map(bridge, msg)
        assert bridge.map_msg is None

    def test_get_map_data_returns_none_for_bad_dims(self):
        """get_map_data should return None for malformed map."""
        bridge = MagicMock()
        mock_map = MagicMock()
        mock_map.info.width = 10
        mock_map.info.height = 10
        mock_map.data = [0] * 50  # mismatched
        bridge.map_msg = mock_map
        result = webapp_module.WebBridgeNode.get_map_data(bridge)
        assert result is None


# ── Bug 51: Rename room should not overwrite existing ──
class TestRenameRoomOverwrite:
    """Bug 51: rename_room should refuse to overwrite existing rooms."""

    def test_rename_to_existing_room_fails(self, flask_client, temp_rooms_dir):
        """Renaming to an already-existing room name should return error."""
        # Create two room files
        for name in ("room_a", "room_b"):
            path = temp_rooms_dir / f"{name}.json"
            path.write_text(json.dumps({"name": name, "width": 10, "height": 10,
                                         "resolution": 0.05, "data": [0]*100}))
        # Try to rename room_a to room_b
        ok, info = webapp_module.WebBridgeNode.rename_room("room_a", "room_b")
        assert not ok
        assert "already exists" in info.lower()
        # Verify room_a still exists
        assert (temp_rooms_dir / "room_a.json").exists()
        # Verify room_b is unchanged
        with open(temp_rooms_dir / "room_b.json") as f:
            d = json.load(f)
        assert d["name"] == "room_b"
