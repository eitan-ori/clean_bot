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
from datetime import datetime
from pathlib import Path

import numpy as np

# Flask
from flask import Flask, render_template, request, jsonify, send_from_directory
from flask_socketio import SocketIO, emit

# ROS 2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

logger = logging.getLogger(__name__)

# ── Paths ───────────────────────────────────────────────────────────
WEBAPP_DIR = Path(__file__).parent
SAVED_ROOMS_DIR = WEBAPP_DIR / "saved_rooms"
SAVED_ROOMS_DIR.mkdir(exist_ok=True)


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
        self.mission_log = []  # list of {time, event} dicts
        self._prev_mission_state = "UNKNOWN"
        self._last_map_emit = 0.0
        self._last_pose_emit = 0.0

        # ── Periodic pose updater (10 Hz) ──
        self.create_timer(0.1, self._update_pose)
        # ── Periodic status push (2 Hz) ──
        self.create_timer(0.5, self._push_status)

        self.get_logger().info("Web control panel ROS bridge started")

    # ── ROS Callbacks ──────────────────────────────────────────────
    def _on_mission_state(self, msg):
        old = self.mission_state
        self.mission_state = msg.data
        if old != msg.data:
            entry = {"time": datetime.now().strftime("%H:%M:%S"), "event": f"Mission: {old} → {msg.data}"}
            self.mission_log.append(entry)
            if len(self.mission_log) > 200:
                self.mission_log = self.mission_log[-200:]
            self.sio.emit("mission_state", {"state": msg.data, "log_entry": entry})

    def _on_exploration_state(self, msg):
        self.exploration_state = msg.data

    def _on_coverage_state(self, msg):
        self.coverage_state = msg.data

    def _on_odom(self, msg):
        self.linear_vel = msg.twist.twist.linear.x
        self.angular_vel = msg.twist.twist.angular.z

    def _on_scan(self, msg):
        self.scan_ranges = list(msg.ranges)
        self.scan_angle_min = msg.angle_min
        self.scan_angle_max = msg.angle_max
        self.scan_angle_increment = msg.angle_increment

    def _on_map(self, msg):
        self.map_msg = msg
        self.map_update_counter += 1
        # Rate-limit map emission to ≤2 Hz
        now = time.monotonic()
        if now - self._last_map_emit >= 0.5:
            self._last_map_emit = now
            self._emit_map()

    # ── Periodic helpers ──────────────────────────────────────────
    def _update_pose(self):
        try:
            t = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            self.robot_x = t.transform.translation.x
            self.robot_y = t.transform.translation.y
            qz = t.transform.rotation.z
            qw = t.transform.rotation.w
            self.robot_yaw = math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)
        except Exception:
            pass

    def _push_status(self):
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

    def send_velocity(self, linear, angular):
        t = Twist()
        t.linear.x = float(linear)
        t.angular.z = float(angular)
        self.vel_pub.publish(t)

    # ── Room save / load ──────────────────────────────────────────
    def save_room(self, name):
        """Save current map data to a JSON file."""
        if self.map_msg is None:
            return False, "No map available"
        m = self.map_msg
        room = {
            "name": name,
            "saved_at": datetime.now().isoformat(),
            "width": m.info.width,
            "height": m.info.height,
            "resolution": m.info.resolution,
            "origin_x": m.info.origin.position.x,
            "origin_y": m.info.origin.position.y,
            "data": list(m.data),
        }
        safe_name = "".join(c if c.isalnum() or c in "-_ " else "" for c in name).strip().replace(" ", "_")
        path = SAVED_ROOMS_DIR / f"{safe_name}.json"
        try:
            with open(path, "w") as f:
                json.dump(room, f)
        except (TypeError, ValueError, OverflowError) as e:
            return False, f"JSON serialization error: {e}"
        return True, str(path)

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
    def delete_room(filename):
        path = SAVED_ROOMS_DIR / f"{filename}.json"
        if path.exists():
            path.unlink()
            return True
        return False

    def load_room_preview(self, filename):
        """Load a saved room and return map image data."""
        path = SAVED_ROOMS_DIR / f"{filename}.json"
        if not path.exists():
            return None
        with open(path) as f:
            d = json.load(f)
        for field in ("width", "height", "data"):
            if field not in d:
                return None
        w, h = d["width"], d["height"]
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
    cmd = request.json.get("command", "")
    valid = {"start_scan", "stop_scan", "start_clean", "stop_clean",
             "go_home", "reset", "pause", "resume"}
    if cmd not in valid:
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
    lin = request.json.get("linear", 0.0)
    ang = request.json.get("angular", 0.0)
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
    name = request.json.get("name", "").strip()
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
    if ros_node is None:
        return jsonify({"error": "ROS not connected"}), 503
    try:
        data = ros_node.load_room_preview(filename)
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


@app.route("/api/log")
def api_log():
    if ros_node is None:
        return jsonify([])
    return jsonify(ros_node.mission_log[-100:])


# ── WebSocket Events ─────────────────────────────────────────────
@socketio.on("connect")
def ws_connect():
    logger.info("WebSocket client connected")
    if ros_node and ros_node.map_msg:
        ros_node._emit_map()


@socketio.on("command")
def ws_command(data):
    if ros_node:
        ros_node.send_command(data.get("command", ""))


@socketio.on("velocity")
def ws_velocity(data):
    if ros_node:
        ros_node.send_velocity(data.get("linear", 0), data.get("angular", 0))


@socketio.on("request_map")
def ws_request_map():
    if ros_node:
        ros_node._emit_map()


# ════════════════════════════════════════════════════════════════════
# Main
# ════════════════════════════════════════════════════════════════════
def main():
    global ros_node

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")

    print("=" * 60)
    print("🤖  Clean Bot Web Control Panel")
    print("=" * 60)
    print()
    print("Starting ROS 2 bridge...")

    if not rclpy.ok():
        rclpy.init()

    ros_node = WebBridgeNode(socketio)
    executor = MultiThreadedExecutor()
    executor.add_node(ros_node)

    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    print("ROS 2 bridge started.")
    print()
    print("Starting web server on http://0.0.0.0:5000")
    print("Open this URL in your browser to control the robot.")
    print("=" * 60)

    try:
        socketio.run(app, host="0.0.0.0", port=5000, allow_unsafe_werkzeug=True)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        executor.shutdown()
        ros_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
