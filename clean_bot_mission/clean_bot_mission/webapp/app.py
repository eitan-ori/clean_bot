#!/usr/bin/env python3
"""
Clean Bot Web Control Panel

A Flask + Flask-SocketIO web application that provides a full-featured
control panel for the cleaning robot, replacing the Telegram bot interface.

Features:
- Live updating map with robot position and obstacles
- Room mapping: scan and save room maps with names
- Room management: list, load, delete saved rooms
- Mission control: scan, clean specific rooms, go home
- Manual joystick control (forward/back/left/right/stop)
- Real-time status dashboard
- Mission history log
- Robot parameter configuration

Architecture:
    Browser <--WebSocket--> Flask-SocketIO <--rclpy--> ROS 2 Topics

Usage:
    python3 -m clean_bot_mission.webapp.app
    # Then open http://localhost:5000
"""

import os
import sys
import json
import time
import math
import threading
import logging
from collections import deque
from datetime import datetime
from pathlib import Path

import numpy as np

# Flask
from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit

# ROS 2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

try:
    from nav2_msgs.action import NavigateToPose as Nav2NavigateToPose
    HAS_NAV2 = True
except ImportError:
    HAS_NAV2 = False

logger = logging.getLogger(__name__)

# ── Paths ───────────────────────────────────────────────────────────
WEBAPP_DIR = Path(__file__).parent
SAVED_ROOMS_DIR = WEBAPP_DIR / "saved_rooms"
SAVED_ROOMS_DIR.mkdir(exist_ok=True)
SCHEDULES_FILE = WEBAPP_DIR / "schedules.json"


# ════════════════════════════════════════════════════════════════════
# ROS 2 Bridge Node
# ════════════════════════════════════════════════════════════════════
class WebBridgeNode(Node):
    """ROS 2 node that bridges the web app to the robot."""

    def __init__(self, socketio_instance):
        super().__init__("web_control_panel")
        self.sio = socketio_instance

        # ── Publishers ──
        self.cmd_pub = self.create_publisher(String, "mission_command", 10)
        self.vel_pub = self.create_publisher(Twist, "cmd_vel_nav", 10)
        # Publisher for injecting saved room maps
        map_pub_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.map_pub = self.create_publisher(OccupancyGrid, "map", map_pub_qos)

        # ── QoS for map ──
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # ── Subscribers ──
        self.create_subscription(String, "mission_state", self._on_mission_state, 10)
        self.create_subscription(String, "exploration_state", self._on_exploration_state, 10)
        self.create_subscription(String, "coverage_state", self._on_coverage_state, 10)
        self.create_subscription(OccupancyGrid, "map", self._on_map, map_qos)
        self.create_subscription(Odometry, "odom", self._on_odom, 10)
        self.create_subscription(LaserScan, "scan", self._on_scan, 10)

        # ── TF ──
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ── State ──
        self.mission_state = "UNKNOWN"
        self.exploration_state = "UNKNOWN"
        self.coverage_state = "UNKNOWN"
        self.map_msg = None
        self.map_update_counter = 0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.scan_ranges = []
        self.scan_angle_min = 0.0
        self.scan_angle_max = 0.0
        self.scan_angle_increment = 0.0
        self.mission_log = deque(maxlen=200)  # thread-safe, auto-truncating
        self._prev_mission_state = "UNKNOWN"
        self._last_map_emit = 0.0
        self._last_pose_emit = 0.0

        # ── Round 31: Statistics ──
        self.total_scan_time = 0.0
        self.total_clean_time = 0.0
        self.rooms_scanned = 0
        self.rooms_cleaned = 0
        self.total_distance_traveled = 0.0
        self._scan_start_time = None
        self._clean_start_time = None
        self._last_odom_x = None
        self._last_odom_y = None

        # ── Round 32: Obstacle heatmap ──
        self.obstacle_heatmap = None
        self._heatmap_width = 0
        self._heatmap_height = 0

        # ── Round 33: Schedules ──
        self._schedules = self._load_schedules()
        self._schedule_lock = threading.Lock()

        # ── Round 34: Battery simulation ──
        self.battery_level = 100.0
        self._battery_last_update = time.monotonic()
        self._arduino_battery = None

        # ── Round 35: Path trail ──
        self.path_trail = []
        self._trail_sent_index = 0
        self._trail_lock = threading.Lock()

        # ── Round 39: Configurable map rate ──
        self._map_rate_interval = 0.5

        # ── Round 40: Diagnostics ──
        self._scan_callback_count = 0
        self._scan_hz_last_time = time.monotonic()
        self.scan_hz = 0.0
        self._scan_valid_count = 0
        self._tf_healthy = False
        self._missed_updates = 0

        # ── Nav2 action client (Round 14) ──
        self._nav_client = None
        if HAS_NAV2:
            self._nav_client = ActionClient(self, Nav2NavigateToPose, 'navigate_to_pose')

        # ── Periodic pose updater (10 Hz) ──
        self.create_timer(0.1, self._update_pose)
        # ── Periodic status push (2 Hz) ──
        self.create_timer(0.5, self._push_status)
        # ── Round 33: Schedule checker (every 60s) ──
        self.create_timer(60.0, self._check_schedules)
        # ── Round 40: Scan Hz calculator (every 1s) ──
        self.create_timer(1.0, self._calc_scan_hz)

        self.get_logger().info("Web control panel ROS bridge started")

    # ── ROS Callbacks ──────────────────────────────────────────────
    def _on_mission_state(self, msg):
        old = self.mission_state
        self.mission_state = msg.data
        if old != msg.data:
            entry = {"time": datetime.now().strftime("%H:%M:%S"), "event": f"Mission: {old} → {msg.data}"}
            self.mission_log.append(entry)
            self.sio.emit("mission_state", {"state": msg.data, "log_entry": entry})
            # Round 31: Track scan/clean stats
            old_u = (old or '').upper()
            new_u = (msg.data or '').upper()
            if 'EXPLOR' in new_u and 'EXPLOR' not in old_u:
                self._scan_start_time = time.monotonic()
            elif 'EXPLOR' not in new_u and 'EXPLOR' in old_u and self._scan_start_time:
                self.total_scan_time += time.monotonic() - self._scan_start_time
                self.rooms_scanned += 1
                self._scan_start_time = None
            if ('COVER' in new_u or 'CLEAN' in new_u) and 'COVER' not in old_u and 'CLEAN' not in old_u:
                self._clean_start_time = time.monotonic()
            elif ('COVER' not in new_u and 'CLEAN' not in new_u) and \
                 ('COVER' in old_u or 'CLEAN' in old_u) and self._clean_start_time:
                self.total_clean_time += time.monotonic() - self._clean_start_time
                self.rooms_cleaned += 1
                self._clean_start_time = None

    def _on_exploration_state(self, msg):
        self.exploration_state = msg.data

    def _on_coverage_state(self, msg):
        self.coverage_state = msg.data

    def _on_odom(self, msg):
        self.linear_vel = msg.twist.twist.linear.x
        self.angular_vel = msg.twist.twist.angular.z
        # Round 31: Integrate distance
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self._last_odom_x is not None:
            dx = x - self._last_odom_x
            dy = y - self._last_odom_y
            self.total_distance_traveled += math.sqrt(dx * dx + dy * dy)
        self._last_odom_x = x
        self._last_odom_y = y
        # Round 34: Battery simulation
        now = time.monotonic()
        dt = now - self._battery_last_update
        self._battery_last_update = now
        speed = abs(self.linear_vel) + abs(self.angular_vel) * 0.3
        drain = dt * (0.02 + speed * 0.15)  # base drain + velocity drain
        self.battery_level = max(0.0, self.battery_level - drain)

    def _on_scan(self, msg):
        self.scan_ranges = list(msg.ranges)
        self.scan_angle_min = msg.angle_min
        self.scan_angle_max = msg.angle_max
        self.scan_angle_increment = msg.angle_increment
        # Round 40: Track scan frequency
        self._scan_callback_count += 1
        self._scan_valid_count = sum(1 for r in msg.ranges if math.isfinite(r) and r > 0)

    def _on_map(self, msg):
        self.map_msg = msg
        self.map_update_counter += 1
        # Round 32: Update obstacle heatmap
        w, h = msg.info.width, msg.info.height
        data = np.array(msg.data, dtype=np.int8).reshape((h, w))
        if self.obstacle_heatmap is None or self._heatmap_width != w or self._heatmap_height != h:
            self.obstacle_heatmap = np.zeros((h, w), dtype=np.float32)
            self._heatmap_width = w
            self._heatmap_height = h
        self.obstacle_heatmap[data >= 50] += 1.0
        # Rate-limit map emission
        now = time.monotonic()
        if now - self._last_map_emit >= self._map_rate_interval:
            self._last_map_emit = now
            self._emit_map()

    # ── Periodic helpers ──────────────────────────────────────────
    def _update_pose(self):
        try:
            t = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            self.robot_x = t.transform.translation.x
            self.robot_y = t.transform.translation.y
            q = t.transform.rotation
            self.robot_yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            self._tf_healthy = True
            # Round 35: Path trail
            with self._trail_lock:
                trail = self.path_trail
                if not trail or (abs(self.robot_x - trail[-1][0]) > 0.01 or
                                 abs(self.robot_y - trail[-1][1]) > 0.01):
                    trail.append((round(self.robot_x, 3), round(self.robot_y, 3)))
                    if len(trail) > 500:
                        self.path_trail = trail[-500:]
        except Exception:
            self._tf_healthy = False

    def _push_status(self):
        # Round 34: Use arduino battery if available
        battery = self._arduino_battery if self._arduino_battery is not None else round(self.battery_level, 1)
        # Round 35: Only send new trail points since last push
        with self._trail_lock:
            new_trail = self.path_trail[self._trail_sent_index:]
            self._trail_sent_index = len(self.path_trail)
        self.sio.emit("status", {
            "mission_state": self.mission_state,
            "exploration_state": self.exploration_state,
            "coverage_state": self.coverage_state,
            "robot_x": round(self.robot_x, 3),
            "robot_y": round(self.robot_y, 3),
            "robot_yaw": round(self.robot_yaw, 3),
            "linear_vel": round(self.linear_vel, 3),
            "angular_vel": round(self.angular_vel, 3),
            "map_updates": self.map_update_counter,
            "battery": battery,
            "new_trail": new_trail,
            "trail_total": len(self.path_trail),
            # Round 40: Diagnostics
            "scan_hz": round(self.scan_hz, 1),
            "scan_valid_count": self._scan_valid_count,
            "total_distance": round(self.total_distance_traveled, 2),
            "tf_healthy": self._tf_healthy,
        })

    # ── Map rendering ─────────────────────────────────────────────
    def get_map_data(self):
        """Return map as a dict ready for JSON: pixels, metadata, robot pose, obstacles."""
        if self.map_msg is None:
            return None
        m = self.map_msg
        w, h = m.info.width, m.info.height
        res = m.info.resolution
        ox = m.info.origin.position.x
        oy = m.info.origin.position.y
        data = np.array(m.data, dtype=np.int8).reshape((h, w))

        # Build RGBA pixel buffer (sent as flat list, rendered on canvas)
        # -1=unknown(gray), 0=free(white), 1-49=light gray, 50-100=black(obstacle)
        rgba = np.zeros((h, w, 4), dtype=np.uint8)
        rgba[..., 3] = 255  # opaque
        unknown = data == -1
        free = data == 0
        occupied = data >= 50
        partial = (~unknown) & (~free) & (~occupied)

        rgba[unknown] = [40, 40, 48, 255]
        rgba[free] = [230, 235, 240, 255]
        rgba[occupied] = [220, 50, 50, 255]  # Red for obstacles
        if np.any(partial):
            vals = data[partial].astype(np.float32) / 50.0
            g = (230 - 180 * vals).astype(np.uint8)
            rgba[partial, 0] = g
            rgba[partial, 1] = g
            rgba[partial, 2] = g

        # Flip vertically (ROS origin = bottom-left)
        rgba = rgba[::-1]

        # Collect obstacle points (for overlay markers)
        occ_y, occ_x = np.where(data >= 50)
        # Downsample obstacles for perf (send max ~2000 points)
        if len(occ_x) > 2000:
            idx = np.random.choice(len(occ_x), 2000, replace=False)
            occ_x, occ_y = occ_x[idx], occ_y[idx]

        # Convert to world coords
        obs_world = []
        for px_x, px_y in zip(occ_x.tolist(), occ_y.tolist()):
            wx = ox + px_x * res
            wy = oy + px_y * res
            obs_world.append([round(wx, 3), round(wy, 3)])

        # Robot pixel position
        rpx = (self.robot_x - ox) / res if res > 0 else 0
        rpy = h - 1 - (self.robot_y - oy) / res if res > 0 else 0

        import base64, io
        from PIL import Image
        img = Image.fromarray(rgba, "RGBA")
        # Scale up small maps for clarity
        min_dim = 500
        if w < min_dim or h < min_dim:
            scale = max(min_dim / w, min_dim / h, 1.0)
            img = img.resize((int(w * scale), int(h * scale)), Image.NEAREST)
        buf = io.BytesIO()
        img.save(buf, format="PNG", optimize=True)
        b64 = base64.b64encode(buf.getvalue()).decode("ascii")

        # Round 32: Heatmap data (vectorized with numpy)
        heatmap_b64 = None
        if self.obstacle_heatmap is not None and self._heatmap_width == w and self._heatmap_height == h:
            hm = self.obstacle_heatmap
            max_val = hm.max()
            if max_val > 0:
                hm_norm = (hm / max_val * 255).astype(np.uint8)
                hm_norm = hm_norm[::-1]
                hm_rgba = np.zeros((h, w, 4), dtype=np.uint8)
                mask = hm_norm > 0
                v = hm_norm[mask].astype(np.int16)
                low = v < 128
                mid = (v >= 128) & (v < 200)
                high = v >= 200
                r = np.zeros_like(v, dtype=np.uint8)
                g = np.zeros_like(v, dtype=np.uint8)
                b = np.zeros_like(v, dtype=np.uint8)
                a = np.zeros_like(v, dtype=np.uint8)
                r[low] = 0; g[low] = 0
                b[low] = np.minimum(255, v[low] * 4).astype(np.uint8)
                a[low] = np.minimum(200, v[low] * 2).astype(np.uint8)
                r[mid] = np.minimum(255, (v[mid] - 128) * 3).astype(np.uint8)
                g[mid] = np.minimum(255, v[mid]).astype(np.uint8)
                b[mid] = 0; a[mid] = np.minimum(220, v[mid]).astype(np.uint8)
                r[high] = 255
                g[high] = np.maximum(0, 255 - (v[high] - 200) * 4).astype(np.uint8)
                b[high] = 0; a[high] = 230
                hm_rgba[mask, 0] = r
                hm_rgba[mask, 1] = g
                hm_rgba[mask, 2] = b
                hm_rgba[mask, 3] = a
                hm_img = Image.fromarray(hm_rgba, "RGBA")
                if w < min_dim or h < min_dim:
                    hm_img = hm_img.resize((int(w * scale), int(h * scale)), Image.NEAREST)
                hm_buf = io.BytesIO()
                hm_img.save(hm_buf, format="PNG", optimize=True)
                heatmap_b64 = base64.b64encode(hm_buf.getvalue()).decode("ascii")

        return {
            "image_b64": b64,
            "width": w,
            "height": h,
            "resolution": res,
            "origin_x": ox,
            "origin_y": oy,
            "robot_px": round(rpx, 1),
            "robot_py": round(rpy, 1),
            "robot_yaw": round(self.robot_yaw, 3),
            "obstacles": obs_world,
            "map_updates": self.map_update_counter,
            "heatmap_b64": heatmap_b64,
        }

    def _emit_map(self):
        data = self.get_map_data()
        if data:
            self.sio.emit("map_update", data)

    # ── Commands ──────────────────────────────────────────────────
    def send_command(self, cmd):
        msg = String()
        msg.data = cmd
        self.cmd_pub.publish(msg)
        entry = {"time": datetime.now().strftime("%H:%M:%S"), "event": f"Command sent: {cmd}"}
        self.mission_log.append(entry)
        self.get_logger().info(f"Command: {cmd}")
        # Safety: stop commands also zero velocity to halt the robot immediately
        if cmd in ("stop_clean", "stop_scan"):
            self.send_velocity(0.0, 0.0)

    def send_velocity(self, linear, angular):
        t = Twist()
        t.linear.x = float(linear)
        t.angular.z = float(angular)
        self.vel_pub.publish(t)

    # ── Room save / load ──────────────────────────────────────────
    def save_room(self, name):
        """Save current map data to a JSON file (walls only — obstacles excluded)."""
        if self.map_msg is None:
            return False, "No map available"
        m = self.map_msg
        # Save only structural walls: occupancy >= 50 → 100 (wall), else → 0 (free)
        # This lets movable obstacles be re-detected on next clean
        raw = list(m.data)
        walls_only = [100 if v >= 50 else 0 for v in raw]
        room = {
            "name": name,
            "saved_at": datetime.now().isoformat(),
            "width": m.info.width,
            "height": m.info.height,
            "resolution": m.info.resolution,
            "origin_x": m.info.origin.position.x,
            "origin_y": m.info.origin.position.y,
            "data": walls_only,
        }
        safe_name = "".join(c if c.isalnum() or c in "-_ " else "" for c in name).strip().replace(" ", "_")
        if not safe_name:
            return False, "Room name must contain at least one alphanumeric character"
        path = SAVED_ROOMS_DIR / f"{safe_name}.json"
        try:
            with open(path, "w") as f:
                json.dump(room, f)
        except (TypeError, ValueError, OverflowError, OSError) as e:
            return False, f"Save error: {e}"
        return True, str(path)

    @staticmethod
    def rename_room(filename, new_name):
        """Rename a saved room: update the JSON name field and rename the file."""
        path = WebBridgeNode._safe_room_path(filename)
        if path is None or not path.exists():
            return False, "Room not found"
        try:
            with open(path) as f:
                d = json.load(f)
            d["name"] = new_name
            safe_new = "".join(c if c.isalnum() or c in "-_ " else "" for c in new_name).strip().replace(" ", "_")
            if not safe_new:
                return False, "New name must contain at least one alphanumeric character"
            new_path = SAVED_ROOMS_DIR / f"{safe_new}.json"
            if new_path != path:
                with open(new_path, "w") as f:
                    json.dump(d, f)
                path.unlink()
            else:
                with open(path, "w") as f:
                    json.dump(d, f)
            return True, safe_new
        except Exception as e:
            return False, str(e)

    def load_and_clean_room(self, filename):
        """Load a saved room map (walls only), publish it, and start cleaning."""
        path = WebBridgeNode._safe_room_path(filename)
        if path is None or not path.exists():
            return False, "Room not found"
        try:
            with open(path) as f:
                d = json.load(f)
        except (json.JSONDecodeError, OSError):
            return False, "Cannot read room file"
        for field in ("width", "height", "data", "resolution"):
            if field not in d:
                return False, f"Room file missing '{field}'"
        w, h = d["width"], d["height"]
        if w <= 0 or h <= 0 or len(d["data"]) != w * h:
            return False, "Invalid room data dimensions"
        # Build an OccupancyGrid from the saved walls-only data
        from nav_msgs.msg import OccupancyGrid as OG
        from geometry_msgs.msg import Pose
        from std_msgs.msg import Header
        msg = OG()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info.width = w
        msg.info.height = h
        msg.info.resolution = float(d["resolution"])
        msg.info.origin = Pose()
        msg.info.origin.position.x = float(d.get("origin_x", 0.0))
        msg.info.origin.position.y = float(d.get("origin_y", 0.0))
        msg.info.origin.orientation.w = 1.0
        msg.data = [int(v) for v in d["data"]]
        # Update internal map state
        self.map_msg = msg
        self.map_update_counter += 1
        # Publish to ROS so SLAM/Nav2/coverage planner pick it up
        self.map_pub.publish(msg)
        self.get_logger().info(f"Published saved room map '{d.get('name', filename)}' ({w}x{h})")
        # Send start_clean command
        self.send_command("start_clean")
        return True, f"Loaded room '{d.get('name', filename)}' and started cleaning"

    def navigate_to_pose(self, x, y):
        """Send a NavigateToPose goal via nav2 action client."""
        if not HAS_NAV2 or self._nav_client is None:
            self.get_logger().warn("Nav2 action client not available")
            return False, "Nav2 not available"
        goal = Nav2NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.w = 1.0
        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            return False, "Nav2 action server not available"
        self._nav_client.send_goal_async(goal)
        entry = {"time": datetime.now().strftime("%H:%M:%S"),
                 "event": f"Navigate to ({x:.2f}, {y:.2f})"}
        self.mission_log.append(entry)
        self.sio.emit("mission_state", {"state": self.mission_state, "log_entry": entry})
        self.get_logger().info(f"Navigate to ({x}, {y})")
        return True, "Goal sent"

    # ── Round 33: Schedule helpers ────────────────────────────────
    @staticmethod
    def _load_schedules():
        if SCHEDULES_FILE.exists():
            try:
                with open(SCHEDULES_FILE) as f:
                    return json.load(f)
            except Exception:
                pass
        return []

    def _save_schedules(self):
        with self._schedule_lock:
            try:
                with open(SCHEDULES_FILE, "w") as f:
                    json.dump(self._schedules, f)
            except Exception as e:
                self.get_logger().warn(f"Failed to save schedules: {e}")

    def _check_schedules(self):
        now = datetime.now()
        day_names = ["mon", "tue", "wed", "thu", "fri", "sat", "sun"]
        current_day = day_names[now.weekday()]
        current_time = now.strftime("%H:%M")
        # Only trigger if robot is idle (waiting states)
        state_upper = (self.mission_state or '').upper()
        if 'WAIT' not in state_upper and 'COMPLETE' not in state_upper and 'UNKNOWN' not in state_upper:
            return
        with self._schedule_lock:
            for sched in self._schedules:
                if not sched.get("enabled", True):
                    continue
                if current_day in sched.get("days", []) and sched.get("time") == current_time:
                    # Prevent double-triggering within the same minute
                    last = sched.get("_last_triggered", "")
                    trigger_key = f"{current_day}_{current_time}"
                    if last == trigger_key:
                        continue
                    sched["_last_triggered"] = trigger_key
                    self.get_logger().info(f"Scheduled clean triggered: {sched}")
                    self.send_command("start_clean")

    # ── Round 40: Scan Hz calculator ─────────────────────────────
    def _calc_scan_hz(self):
        now = time.monotonic()
        dt = now - self._scan_hz_last_time
        if dt > 0:
            self.scan_hz = self._scan_callback_count / dt
        self._scan_callback_count = 0
        self._scan_hz_last_time = now

    # ── Round 31: Get statistics ─────────────────────────────────
    def get_stats(self):
        active_scan = 0.0
        active_clean = 0.0
        if self._scan_start_time:
            active_scan = time.monotonic() - self._scan_start_time
        if self._clean_start_time:
            active_clean = time.monotonic() - self._clean_start_time
        return {
            "total_scan_time": round(self.total_scan_time + active_scan, 1),
            "total_clean_time": round(self.total_clean_time + active_clean, 1),
            "rooms_scanned": self.rooms_scanned,
            "rooms_cleaned": self.rooms_cleaned,
            "total_distance": round(self.total_distance_traveled, 2),
        }

    @staticmethod
    def list_rooms():
        rooms = []
        for p in sorted(SAVED_ROOMS_DIR.glob("*.json")):
            try:
                with open(p) as f:
                    d = json.load(f)
                rooms.append({
                    "filename": p.stem,
                    "name": d.get("name", p.stem),
                    "saved_at": d.get("saved_at", ""),
                    "width": d.get("width", 0),
                    "height": d.get("height", 0),
                    "resolution": d.get("resolution", 0),
                })
            except Exception:
                pass
        return rooms

    @staticmethod
    def _safe_room_path(filename):
        """Return a safe path inside SAVED_ROOMS_DIR, or None if traversal detected."""
        safe = "".join(c if c.isalnum() or c in "-_" else "" for c in filename)
        if not safe or safe != filename:
            return None
        path = (SAVED_ROOMS_DIR / f"{safe}.json").resolve()
        if not str(path).startswith(str(SAVED_ROOMS_DIR.resolve())):
            return None
        return path

    @staticmethod
    def delete_room(filename):
        path = WebBridgeNode._safe_room_path(filename)
        if path is None or not path.exists():
            return False
        path.unlink()
        return True

    @staticmethod
    def load_room_preview(filename):
        """Load a saved room and return map image data."""
        path = WebBridgeNode._safe_room_path(filename)
        if path is None or not path.exists():
            return None
        try:
            with open(path) as f:
                d = json.load(f)
        except (json.JSONDecodeError, OSError):
            return None
        for field in ("width", "height", "data", "resolution"):
            if field not in d:
                return None
        w, h = d["width"], d["height"]
        if w <= 0 or h <= 0 or len(d["data"]) != w * h:
            return None
        res = d["resolution"]
        data = np.array(d["data"], dtype=np.int8).reshape((h, w))

        rgba = np.zeros((h, w, 4), dtype=np.uint8)
        rgba[..., 3] = 255
        rgba[data == -1] = [40, 40, 48, 255]
        rgba[data == 0] = [230, 235, 240, 255]
        rgba[data >= 50] = [220, 50, 50, 255]
        partial = (data > 0) & (data < 50)
        if np.any(partial):
            vals = data[partial].astype(np.float32) / 50.0
            g = (230 - 180 * vals).astype(np.uint8)
            rgba[partial, 0] = g
            rgba[partial, 1] = g
            rgba[partial, 2] = g
        rgba = rgba[::-1]

        import base64, io
        from PIL import Image
        img = Image.fromarray(rgba, "RGBA")
        min_dim = 400
        if w < min_dim or h < min_dim:
            scale = max(min_dim / w, min_dim / h, 1.0)
            img = img.resize((int(w * scale), int(h * scale)), Image.NEAREST)
        buf = io.BytesIO()
        img.save(buf, format="PNG", optimize=True)
        b64 = base64.b64encode(buf.getvalue()).decode("ascii")
        return {
            "image_b64": b64,
            "name": d.get("name", filename),
            "saved_at": d.get("saved_at", ""),
            "width": w, "height": h, "resolution": res,
        }


# ════════════════════════════════════════════════════════════════════
# Flask Application
# ════════════════════════════════════════════════════════════════════
app = Flask(
    __name__,
    template_folder=str(WEBAPP_DIR / "templates"),
    static_folder=str(WEBAPP_DIR / "static"),
)
app.config["SECRET_KEY"] = "cleanbot-web-panel"
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

ros_node: WebBridgeNode = None


VALID_COMMANDS = {"start_scan", "stop_scan", "start_clean", "stop_clean",
                  "go_home", "reset", "pause", "resume"}


# ── HTTP Routes ───────────────────────────────────────────────────
@app.route("/")
def index():
    return render_template("index.html")


@app.route("/api/status")
def api_status():
    if ros_node is None:
        return jsonify({"error": "ROS not connected"}), 503
    try:
        return jsonify({
            "mission_state": ros_node.mission_state,
            "exploration_state": ros_node.exploration_state,
            "coverage_state": ros_node.coverage_state,
            "robot_x": ros_node.robot_x,
            "robot_y": ros_node.robot_y,
            "robot_yaw": ros_node.robot_yaw,
            "linear_vel": ros_node.linear_vel,
            "angular_vel": ros_node.angular_vel,
            "map_updates": ros_node.map_update_counter,
        })
    except Exception as e:
        return jsonify({"error": str(e)}), 500


@app.route("/api/command", methods=["POST"])
def api_command():
    if ros_node is None:
        return jsonify({"error": "ROS not connected"}), 503
    body = request.json or {}
    cmd = body.get("command", "")
    if cmd not in VALID_COMMANDS:
        return jsonify({"error": f"Unknown command: {cmd}"}), 400
    try:
        ros_node.send_command(cmd)
    except Exception as e:
        return jsonify({"error": str(e)}), 500
    return jsonify({"ok": True, "command": cmd})


@app.route("/api/velocity", methods=["POST"])
def api_velocity():
    if ros_node is None:
        return jsonify({"error": "ROS not connected"}), 503
    body = request.json or {}
    lin = body.get("linear", 0.0)
    ang = body.get("angular", 0.0)
    try:
        ros_node.send_velocity(lin, ang)
    except Exception as e:
        return jsonify({"error": str(e)}), 500
    return jsonify({"ok": True})


@app.route("/api/map")
def api_map():
    if ros_node is None:
        return jsonify({"error": "ROS not connected"}), 503
    try:
        data = ros_node.get_map_data()
    except Exception as e:
        return jsonify({"error": str(e)}), 500
    if data is None:
        return jsonify({"error": "No map available"}), 404
    return jsonify(data)


@app.route("/api/rooms")
def api_rooms():
    return jsonify(WebBridgeNode.list_rooms())


@app.route("/api/rooms/save", methods=["POST"])
def api_save_room():
    if ros_node is None:
        return jsonify({"error": "ROS not connected"}), 503
    body = request.json or {}
    name = body.get("name", "").strip()
    if not name:
        return jsonify({"error": "Room name required"}), 400
    try:
        ok, info = ros_node.save_room(name)
    except Exception as e:
        return jsonify({"error": str(e)}), 500
    if ok:
        return jsonify({"ok": True, "path": info})
    return jsonify({"error": info}), 400


@app.route("/api/rooms/<filename>/preview")
def api_room_preview(filename):
    try:
        data = WebBridgeNode.load_room_preview(filename)
    except Exception as e:
        return jsonify({"error": str(e)}), 500
    if data is None:
        return jsonify({"error": "Room not found"}), 404
    return jsonify(data)


@app.route("/api/rooms/<filename>", methods=["DELETE"])
def api_delete_room(filename):
    if WebBridgeNode.delete_room(filename):
        return jsonify({"ok": True})
    return jsonify({"error": "Room not found"}), 404


@app.route("/api/rooms/<filename>/rename", methods=["PUT"])
def api_rename_room(filename):
    body = request.json or {}
    new_name = body.get("name", "").strip()
    if not new_name:
        return jsonify({"error": "New name required"}), 400
    ok, info = WebBridgeNode.rename_room(filename, new_name)
    if ok:
        return jsonify({"ok": True, "new_filename": info})
    return jsonify({"error": info}), 404


@app.route("/api/rooms/<filename>/load_and_clean", methods=["POST"])
def api_load_and_clean(filename):
    if ros_node is None:
        return jsonify({"error": "ROS not connected"}), 503
    try:
        ok, info = ros_node.load_and_clean_room(filename)
    except Exception as e:
        return jsonify({"error": str(e)}), 500
    if ok:
        return jsonify({"ok": True, "message": info})
    return jsonify({"error": info}), 400


@app.route("/api/navigate", methods=["POST"])
def api_navigate():
    if ros_node is None:
        return jsonify({"error": "ROS not connected"}), 503
    body = request.json or {}
    x = body.get("x", 0.0)
    y = body.get("y", 0.0)
    try:
        ok, info = ros_node.navigate_to_pose(x, y)
    except Exception as e:
        return jsonify({"error": str(e)}), 500
    if ok:
        return jsonify({"ok": True, "message": info})
    return jsonify({"error": info}), 400


@app.route("/api/log")
def api_log():
    if ros_node is None:
        return jsonify([])
    return jsonify(list(ros_node.mission_log)[-100:])


@app.route("/api/stats")
def api_stats():
    if ros_node is None:
        return jsonify({"error": "ROS not connected"}), 503
    return jsonify(ros_node.get_stats())


@app.route("/api/trail")
def api_trail():
    if ros_node is None:
        return jsonify([])
    with ros_node._trail_lock:
        return jsonify(ros_node.path_trail[-500:])


@app.route("/api/schedules", methods=["GET"])
def api_get_schedules():
    if ros_node is None:
        return jsonify([])
    with ros_node._schedule_lock:
        return jsonify(ros_node._schedules)


@app.route("/api/schedules", methods=["POST"])
def api_add_schedule():
    if ros_node is None:
        return jsonify({"error": "ROS not connected"}), 503
    body = request.json or {}
    sched_time = body.get("time", "")
    days = body.get("days", [])
    if not sched_time or not days:
        return jsonify({"error": "Time and days required"}), 400
    sched = {
        "id": str(int(time.time() * 1000)),
        "time": sched_time,
        "days": days,
        "enabled": True,
        "created": datetime.now().isoformat(),
    }
    with ros_node._schedule_lock:
        ros_node._schedules.append(sched)
    ros_node._save_schedules()
    return jsonify({"ok": True, "schedule": sched})


@app.route("/api/schedules/<schedule_id>", methods=["DELETE"])
def api_delete_schedule(schedule_id):
    if ros_node is None:
        return jsonify({"error": "ROS not connected"}), 503
    with ros_node._schedule_lock:
        ros_node._schedules = [s for s in ros_node._schedules if s.get("id") != schedule_id]
    ros_node._save_schedules()
    return jsonify({"ok": True})


# ── WebSocket Events ─────────────────────────────────────────────
@socketio.on("connect")
def ws_connect():
    logger.info("WebSocket client connected")
    if ros_node and ros_node.map_msg:
        ros_node._emit_map()


@socketio.on("command")
def ws_command(data):
    if ros_node:
        cmd = data.get("command", "")
        if cmd in VALID_COMMANDS:
            ros_node.send_command(cmd)


@socketio.on("velocity")
def ws_velocity(data):
    if ros_node:
        ros_node.send_velocity(data.get("linear", 0), data.get("angular", 0))


@socketio.on("request_map")
def ws_request_map():
    if ros_node:
        ros_node._emit_map()


@socketio.on("latency_ping")
def ws_latency_ping(data):
    emit("latency_pong", {"ts": data.get("ts", 0)})


@socketio.on("set_map_rate")
def ws_set_map_rate(data):
    if ros_node:
        rate = max(0.5, min(5.0, float(data.get("rate", 0.5))))
        ros_node._map_rate_interval = rate


# ════════════════════════════════════════════════════════════════════
# Main
# ════════════════════════════════════════════════════════════════════
def main():
    global ros_node

    import argparse
    parser = argparse.ArgumentParser(description="Clean Bot Web Control Panel")
    parser.add_argument("--port", type=int, default=5000, help="Web server port (default: 5000)")
    parser.add_argument("--host", default="0.0.0.0", help="Bind address (default: 0.0.0.0)")
    # Parse known args only — ROS 2 may inject its own args
    args, _ = parser.parse_known_args()

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")

    print("=" * 60)
    print("🤖  Clean Bot Web Control Panel")
    print("=" * 60)
    print()

    # Show network info so the user knows how to connect
    try:
        import socket as sock
        hostname = sock.gethostname()
        # Get all non-loopback IPv4 addresses
        addrs = []
        for info in sock.getaddrinfo(hostname, None, sock.AF_INET):
            addr = info[4][0]
            if not addr.startswith("127."):
                addrs.append(addr)
        if addrs:
            print("Network addresses (use any of these from another PC):")
            for a in sorted(set(addrs)):
                print(f"  → http://{a}:{args.port}")
        else:
            print(f"Listening on http://localhost:{args.port}")
        print()
    except Exception:
        pass

    print("Starting ROS 2 bridge...")

    # ROS_DOMAIN_ID must match the robot's domain
    domain_id = os.environ.get("ROS_DOMAIN_ID", "0")
    print(f"ROS_DOMAIN_ID = {domain_id}")
    print("(Set ROS_DOMAIN_ID env var to match your robot if needed)")
    print()

    try:
        if not rclpy.ok():
            rclpy.init()
    except Exception as e:
        print(f"ERROR: Failed to initialize ROS 2: {e}")
        print("Make sure ROS 2 is installed and sourced on this machine.")
        print("The web app requires ROS 2 (rclpy) to communicate with the robot.")
        sys.exit(1)

    ros_node = WebBridgeNode(socketio)
    executor = MultiThreadedExecutor()
    executor.add_node(ros_node)

    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    print("ROS 2 bridge started.")
    print()
    print(f"Starting web server on http://{args.host}:{args.port}")
    print("Open this URL in your browser to control the robot.")
    print("=" * 60)

    try:
        socketio.run(app, host=args.host, port=args.port, allow_unsafe_werkzeug=True)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        executor.shutdown()
        ros_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
