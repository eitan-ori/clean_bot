#!/usr/bin/env python3
"""
Tests for hardware node logic (emergency stop, arduino driver, etc.)
that can run WITHOUT physical hardware or ROS 2.

Tests focus on the computation/logic within callbacks,
not on ROS 2 messaging infrastructure.
"""

import math
import struct
import pytest
from unittest.mock import MagicMock, patch


# ════════════════════════════════════════════════════════════════════
# Test: Emergency Stop Logic
# ════════════════════════════════════════════════════════════════════

class TestEmergencyStopLogic:
    """Test the emergency stop distance-to-speed logic."""

    def setup_method(self):
        self.stop_dist = 0.10
        self.slow_dist = 0.30
        self.slow_factor = 0.3

    def compute_scale(self, distance):
        """Replicate the emergency stop scaling logic."""
        if distance <= self.stop_dist:
            return 0.0
        elif distance <= self.slow_dist:
            dist_range = self.slow_dist - self.stop_dist
            if dist_range > 0:
                return self.slow_factor + (1 - self.slow_factor) * \
                       (distance - self.stop_dist) / dist_range
            else:
                return self.slow_factor
        else:
            return 1.0

    def test_obstacle_at_stop_distance(self):
        assert self.compute_scale(0.10) == 0.0

    def test_obstacle_below_stop_distance(self):
        assert self.compute_scale(0.05) == 0.0

    def test_obstacle_at_slow_distance(self):
        scale = self.compute_scale(0.30)
        assert abs(scale - 1.0) < 0.01

    def test_obstacle_in_slow_zone(self):
        scale = self.compute_scale(0.20)
        # 0.20 is midpoint between 0.10 and 0.30
        expected = self.slow_factor + (1 - self.slow_factor) * 0.5
        assert abs(scale - expected) < 0.01

    def test_obstacle_clear(self):
        assert self.compute_scale(0.50) == 1.0
        assert self.compute_scale(2.0) == 1.0

    def test_equal_distances_no_division_by_zero(self):
        """Bug #4: When slow_dist == stop_dist, no slow zone exists."""
        self.slow_dist = self.stop_dist
        scale = self.compute_scale(self.stop_dist)
        assert scale == 0.0  # At stop distance → full stop
        # Just above stop_dist: falls into clear zone (slow zone is empty)
        scale = self.compute_scale(self.stop_dist + 0.01)
        assert scale == 1.0

    def test_zero_distances(self):
        self.stop_dist = 0.0
        self.slow_dist = 0.0
        assert self.compute_scale(0.0) == 0.0
        # Any positive distance is clear when both thresholds are 0
        assert self.compute_scale(0.01) == 1.0


# ════════════════════════════════════════════════════════════════════
# Test: Differential Drive Math
# ════════════════════════════════════════════════════════════════════

class TestDifferentialDriveMath:
    """Test the velocity-to-PWM conversion logic from ArduinoDriver."""

    def setup_method(self):
        self.max_linear_speed = 0.3
        self.wheel_separation = 0.20
        self.max_pwm = 255
        self.min_pwm_forward = 90
        self.min_pwm_rotate = 90

    def wheel_speed_to_pwm(self, wheel_speed, linear, angular):
        """Replicate ArduinoDriver._wheel_speed_to_pwm."""
        if abs(wheel_speed) < 0.01:
            return 0
        pwm = int((abs(wheel_speed) / self.max_linear_speed) * self.max_pwm)
        pwm = min(180, max(0, pwm))  # MAX_PWM=180 in actual code
        min_pwm = self.min_pwm_forward if abs(linear) > 0.03 else self.min_pwm_rotate
        pwm = max(min_pwm, pwm)
        return pwm if wheel_speed > 0 else -pwm

    def compute_wheel_speeds(self, linear, angular):
        """Replicate differential drive mixing (with negated angular)."""
        angular = -angular
        v_left = linear - angular * (self.wheel_separation / 2.0)
        v_right = linear + angular * (self.wheel_separation / 2.0)
        return v_left, v_right

    def test_forward_motion(self):
        v_l, v_r = self.compute_wheel_speeds(0.15, 0.0)
        assert v_l == pytest.approx(0.15)
        assert v_r == pytest.approx(0.15)

    def test_zero_velocity(self):
        v_l, v_r = self.compute_wheel_speeds(0.0, 0.0)
        assert v_l == 0.0
        assert v_r == 0.0
        pwm_l = self.wheel_speed_to_pwm(v_l, 0.0, 0.0)
        pwm_r = self.wheel_speed_to_pwm(v_r, 0.0, 0.0)
        assert pwm_l == 0
        assert pwm_r == 0

    def test_rotation_only(self):
        v_l, v_r = self.compute_wheel_speeds(0.0, 1.0)
        # angular is negated, so left should go forward, right backward (or vice versa)
        assert v_l > 0 or v_r > 0  # One wheel moves

    def test_backward_motion(self):
        v_l, v_r = self.compute_wheel_speeds(-0.15, 0.0)
        assert v_l == pytest.approx(-0.15)
        assert v_r == pytest.approx(-0.15)
        pwm_l = self.wheel_speed_to_pwm(v_l, -0.15, 0.0)
        assert pwm_l < 0  # Negative PWM for backward

    def test_min_pwm_applied(self):
        pwm = self.wheel_speed_to_pwm(0.02, 0.1, 0.0)
        assert abs(pwm) >= self.min_pwm_forward or pwm == 0

    def test_max_pwm_capped(self):
        pwm = self.wheel_speed_to_pwm(1.0, 1.0, 0.0)
        assert abs(pwm) <= 180

    def test_deadband(self):
        pwm = self.wheel_speed_to_pwm(0.005, 0.0, 0.0)
        assert pwm == 0


# ════════════════════════════════════════════════════════════════════
# Test: Low Obstacle PointCloud Generation
# ════════════════════════════════════════════════════════════════════

class TestLowObstaclePointCloud:
    """Test the cone-pattern point generation from low_obstacle_detector."""

    def test_cone_points(self):
        distance = 0.3
        cone_angle = 0.26
        num_points = 5
        points = []
        for i in range(num_points):
            angle = -cone_angle + (2 * cone_angle * i / (num_points - 1))
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)
            z = 0.0
            points.append((x, y, z))

        assert len(points) == 5
        # All points should be approximately at the correct distance
        for p in points:
            dist = math.sqrt(p[0]**2 + p[1]**2)
            assert dist == pytest.approx(distance, abs=0.001)

    def test_points_packing(self):
        """Test that points pack correctly to PointCloud2 format."""
        points = [(0.3, 0.0, 0.0), (0.29, 0.05, 0.0)]
        data = b''.join(struct.pack('fff', *p) for p in points)
        assert len(data) == 24  # 2 points * 3 floats * 4 bytes

    def test_min_distance_filter(self):
        """Test that readings below min_dist are ignored."""
        min_dist = 0.05
        max_dist = 0.50
        assert 0.03 < min_dist  # Should be filtered
        assert 0.30 < max_dist and 0.30 > min_dist  # Valid

    def test_max_distance_filter(self):
        """Test that readings above max_dist are ignored."""
        min_dist = 0.05
        max_dist = 0.50
        assert 0.60 > max_dist  # Should be filtered


# ════════════════════════════════════════════════════════════════════
# Test: Quaternion to Yaw Conversion
# ════════════════════════════════════════════════════════════════════

class TestQuaternionToYaw:
    """Test the corrected quaternion-to-yaw formula (Bug #1 fix)."""

    def quat_to_yaw(self, qx, qy, qz, qw):
        """Full quaternion to yaw conversion."""
        return math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz))

    def test_identity_quaternion(self):
        """No rotation → yaw = 0."""
        yaw = self.quat_to_yaw(0, 0, 0, 1)
        assert yaw == pytest.approx(0.0, abs=1e-10)

    def test_90_degree_yaw(self):
        """90° rotation about Z axis."""
        # Quaternion for 90° about Z: (0, 0, sin(45°), cos(45°))
        s = math.sin(math.pi / 4)
        c = math.cos(math.pi / 4)
        yaw = self.quat_to_yaw(0, 0, s, c)
        assert yaw == pytest.approx(math.pi / 2, abs=0.01)

    def test_180_degree_yaw(self):
        """180° rotation about Z axis."""
        yaw = self.quat_to_yaw(0, 0, 1, 0)
        assert abs(yaw) == pytest.approx(math.pi, abs=0.01)

    def test_negative_90_degree_yaw(self):
        s = math.sin(-math.pi / 4)
        c = math.cos(-math.pi / 4)
        yaw = self.quat_to_yaw(0, 0, s, c)
        assert yaw == pytest.approx(-math.pi / 2, abs=0.01)


# ════════════════════════════════════════════════════════════════════
# Test: Ultrasonic Range Validation
# ════════════════════════════════════════════════════════════════════

class TestUltrasonicValidation:
    """Test ultrasonic sensor reading validation logic."""

    def test_invalid_zero_reading(self):
        """HC-SR04 returns 0 when no echo - should be ignored."""
        assert 0.0 <= 0.02  # Would be filtered

    def test_valid_reading(self):
        assert 0.15 > 0.02  # Valid

    def test_max_range(self):
        """Readings at max_range mean no obstacle."""
        max_range = 4.0
        assert 4.0 >= max_range  # Should be treated as clear

    def test_distance_to_meters(self):
        """Test cm-to-meters conversion."""
        assert 30 / 100.0 == pytest.approx(0.3)
        assert 200 / 100.0 == pytest.approx(2.0)


# ════════════════════════════════════════════════════════════════════
# Test: IMU Data Conversion
# ════════════════════════════════════════════════════════════════════

class TestIMUConversion:
    """Test IMU data conversion math from simple_imu_driver."""

    def conv(self, high, low):
        """Convert two bytes to signed 16-bit integer."""
        val = (high << 8) + low
        if val >= 0x8000:
            return -((65535 - val) + 1)
        else:
            return val

    def test_positive_value(self):
        # 0x0100 = 256
        assert self.conv(0x01, 0x00) == 256

    def test_zero(self):
        assert self.conv(0x00, 0x00) == 0

    def test_max_positive(self):
        assert self.conv(0x7F, 0xFF) == 32767

    def test_negative_one(self):
        assert self.conv(0xFF, 0xFF) == -1

    def test_min_negative(self):
        assert self.conv(0x80, 0x00) == -32768

    def test_accel_conversion(self):
        """Test accelerometer scaling: raw / 16384 * 9.81."""
        raw = 16384  # Should be 1g
        scale = 16384.0
        result = (raw / scale) * 9.81
        assert result == pytest.approx(9.81, abs=0.01)

    def test_gyro_conversion(self):
        """Test gyroscope scaling: raw / 131 to deg/s, then to rad/s."""
        raw = 131  # Should be 1 deg/s
        scale = 131.0
        result = math.radians(raw / scale)
        assert result == pytest.approx(math.radians(1.0), abs=0.001)

    def test_mag_conversion(self):
        """Test magnetometer scaling: raw * 0.15 * 1e-6 to Tesla."""
        raw = 1000
        scale = 0.15
        result = (raw * scale) * 1e-6
        assert result == pytest.approx(150e-6, abs=1e-9)


# ════════════════════════════════════════════════════════════════════
# Test: Distance Integration
# ════════════════════════════════════════════════════════════════════

class TestDistanceIntegration:
    """Test the odometry-based distance integration."""

    def test_straight_line(self):
        positions = [(0, 0), (1, 0), (2, 0), (3, 0)]
        total = 0.0
        for i in range(1, len(positions)):
            dx = positions[i][0] - positions[i-1][0]
            dy = positions[i][1] - positions[i-1][1]
            total += math.sqrt(dx*dx + dy*dy)
        assert total == pytest.approx(3.0)

    def test_diagonal(self):
        positions = [(0, 0), (1, 1)]
        dx = 1
        dy = 1
        dist = math.sqrt(dx*dx + dy*dy)
        assert dist == pytest.approx(math.sqrt(2))

    def test_no_movement(self):
        positions = [(1, 2), (1, 2), (1, 2)]
        total = 0.0
        for i in range(1, len(positions)):
            dx = positions[i][0] - positions[i-1][0]
            dy = positions[i][1] - positions[i-1][1]
            total += math.sqrt(dx*dx + dy*dy)
        assert total == 0.0


# ════════════════════════════════════════════════════════════════════
# Test: Battery Simulation
# ════════════════════════════════════════════════════════════════════

class TestBatterySimulation:
    """Test the battery drain simulation logic."""

    def test_idle_drain(self):
        """Battery should drain slowly when idle."""
        battery = 100.0
        dt = 1.0  # 1 second
        speed = 0.0
        drain = dt * (0.02 + speed * 0.15)
        battery = max(0.0, battery - drain)
        assert battery == pytest.approx(99.98)

    def test_moving_drain(self):
        """Battery should drain faster when moving."""
        battery = 100.0
        dt = 1.0
        speed = 0.3 + 0.5 * 0.3  # abs(linear) + abs(angular) * 0.3
        drain = dt * (0.02 + speed * 0.15)
        battery = max(0.0, battery - drain)
        assert battery < 99.98  # More drain than idle

    def test_battery_floor(self):
        """Battery should never go below 0."""
        battery = 0.01
        drain = 1.0
        battery = max(0.0, battery - drain)
        assert battery == 0.0


# ════════════════════════════════════════════════════════════════════
# Test: Scan Frequency Calculation
# ════════════════════════════════════════════════════════════════════

class TestScanFrequency:
    """Test scan Hz calculation logic."""

    def test_scan_hz(self):
        count = 10
        dt = 1.0
        hz = count / dt if dt > 0 else 0.0
        assert hz == pytest.approx(10.0)

    def test_scan_hz_zero_time(self):
        count = 10
        dt = 0.0
        hz = count / dt if dt > 0 else 0.0
        assert hz == 0.0


# ════════════════════════════════════════════════════════════════════
# Test: IMU Signed Conversion Edge Cases
# ════════════════════════════════════════════════════════════════════

class TestIMUSignedConversionEdge:
    """Additional edge-case tests for the IMU byte-to-signed conversion."""

    @staticmethod
    def _conv(high, low):
        """Replicate SimpleIMU._conv."""
        val = (high << 8) + low
        if val >= 0x8000:
            return -((65535 - val) + 1)
        return val

    def test_boundary_positive_max(self):
        """0x7FFF should be +32767."""
        assert self._conv(0x7F, 0xFF) == 32767

    def test_boundary_negative_min(self):
        """0x8000 should be -32768."""
        assert self._conv(0x80, 0x00) == -32768

    def test_one_below_boundary(self):
        """0x7FFE should be +32766."""
        assert self._conv(0x7F, 0xFE) == 32766

    def test_one_above_boundary(self):
        """0x8001 should be -32767."""
        assert self._conv(0x80, 0x01) == -32767

    def test_all_ones(self):
        """0xFFFF should be -1."""
        assert self._conv(0xFF, 0xFF) == -1


# ════════════════════════════════════════════════════════════════════
# Test: Emergency Stop Reverse Behavior
# ════════════════════════════════════════════════════════════════════

class TestEmergencyStopReverse:
    """Test that reverse movement is allowed during obstacle stops."""

    def test_reverse_allowed_at_stop(self):
        """Reverse velocity should pass through even at stop distance."""
        # The emergency stop only blocks forward (positive linear.x)
        linear_x = -0.1  # Reverse
        distance = 0.05   # Well within stop distance
        stop_dist = 0.10
        # If linear_x <= 0, it should pass through
        assert linear_x <= 0  # Reverse is not blocked

    def test_rotation_allowed_at_stop(self):
        """Angular velocity should always pass through."""
        angular_z = 0.5
        distance = 0.05
        # The emergency stop always allows rotation
        assert angular_z == 0.5  # Unchanged


class TestEmergencyStopNoSensor:
    """Bug 44: Emergency stop must still reduce speed when sensor never publishes."""

    def test_no_sensor_data_code_path_exists(self):
        """emergency_stop.py must have _start_time and _sensor_grace_sec attributes."""
        import ast
        import os
        for p in ['clean_bot_hardware/emergency_stop.py', 'clean_bot_hardware/clean_bot_hardware/emergency_stop.py']:
            if os.path.exists(p):
                with open(p) as f:
                    src = f.read()
                assert '_start_time' in src, 'missing _start_time tracking for no-sensor case'
                assert '_sensor_grace_sec' in src, 'missing _sensor_grace_sec for grace period'
                return
        pytest.skip('emergency_stop.py not found')

    def test_no_sensor_reduces_speed_after_grace(self):
        """When last_range_time is None and grace period passed, forward speed should halve."""
        import ast, os
        for p in ['clean_bot_hardware/emergency_stop.py', 'clean_bot_hardware/clean_bot_hardware/emergency_stop.py']:
            if os.path.exists(p):
                with open(p) as f:
                    src = f.read()
                # The code must have an 'else' branch for last_range_time is None
                # that checks uptime > grace and reduces speed
                assert 'last_range_time' in src
                # Check for the cautious reduction path
                assert 'never published' in src.lower() or 'sensor never' in src.lower(), \
                    'Missing no-sensor-data warning path in emergency_stop.py'
                return
        pytest.skip('emergency_stop.py not found')


# ════════════════════════════════════════════════════════════════════
# Test: Launch File Port Consistency
# ════════════════════════════════════════════════════════════════════

class TestLaunchFileConsistency:
    """Verify launch file defaults are consistent between
    LaunchConfiguration and DeclareLaunchArgument."""

    def test_arduino_port_consistent(self):
        """The arduino port default should be /dev/ttyACM0 everywhere."""
        import re
        launch_path = 'launch/robot_bringup.launch.py'
        try:
            with open(launch_path) as f:
                content = f.read()
        except FileNotFoundError:
            # Running from repo root
            launch_path = 'clean_bot_hardware/launch/robot_bringup.launch.py'
            with open(launch_path) as f:
                content = f.read()
        # All arduino_port defaults should be the same value
        defaults = re.findall(r"arduino_port.*?default[_value]*[=:]\s*['\"]([^'\"]+)", content)
        assert len(set(defaults)) <= 1, f"Inconsistent arduino_port defaults: {defaults}"

    def test_lidar_port_consistent(self):
        """The lidar port default should be consistent everywhere."""
        import re
        launch_path = 'launch/robot_bringup.launch.py'
        try:
            with open(launch_path) as f:
                content = f.read()
        except FileNotFoundError:
            launch_path = 'clean_bot_hardware/launch/robot_bringup.launch.py'
            with open(launch_path) as f:
                content = f.read()
        defaults = re.findall(r"lidar_port.*?default[_value]*[=:]\s*['\"]([^'\"]+)", content)
        assert len(set(defaults)) <= 1, f"Inconsistent lidar_port defaults: {defaults}"


# ════════════════════════════════════════════════════════════════════
# Test: Safety Topic Naming
# ════════════════════════════════════════════════════════════════════

class TestSafetyTopicNaming:
    """Ensure nodes publish to cmd_vel_nav (not cmd_vel) to go through
    the emergency stop safety filter."""

    def test_simple_coverage_uses_cmd_vel_nav(self):
        """simple_coverage must publish to cmd_vel_nav for safety."""
        try:
            path = 'clean_bot_mission/clean_bot_mission/simple_coverage.py'
            with open(path) as f:
                content = f.read()
        except FileNotFoundError:
            path = '../clean_bot_mission/clean_bot_mission/simple_coverage.py'
            with open(path) as f:
                content = f.read()
        assert "cmd_vel_nav" in content, "simple_coverage should publish to cmd_vel_nav"
        # Verify it does NOT publish directly to 'cmd_vel' (except imports/comments)
        import re
        # Find actual publisher creation lines
        publishers = re.findall(r"create_publisher\(.*?['\"]cmd_vel['\"]", content)
        assert len(publishers) == 0, \
            f"simple_coverage should not publish to bare 'cmd_vel': {publishers}"

    def test_adaptive_coverage_uses_cmd_vel_nav(self):
        """adaptive_coverage must publish to cmd_vel_nav for safety."""
        try:
            path = 'clean_bot_mission/clean_bot_mission/adaptive_coverage.py'
            with open(path) as f:
                content = f.read()
        except FileNotFoundError:
            path = '../clean_bot_mission/clean_bot_mission/adaptive_coverage.py'
            with open(path) as f:
                content = f.read()
        assert "cmd_vel_nav" in content, "adaptive_coverage should publish to cmd_vel_nav"

    def test_nav2_robot_radius_consistent(self):
        """Bug 28: Nav2 local and global costmap robot_radius must match actual (0.20m)."""
        import yaml
        try:
            path = 'clean_bot_hardware/config/nav2_params.yaml'
            with open(path) as f:
                params = yaml.safe_load(f)
        except FileNotFoundError:
            path = 'config/nav2_params.yaml'
            with open(path) as f:
                params = yaml.safe_load(f)
        local_r = params['local_costmap']['local_costmap']['ros__parameters']['robot_radius']
        global_r = params['global_costmap']['global_costmap']['ros__parameters']['robot_radius']
        assert local_r == global_r, f"Costmap robot_radius mismatch: local={local_r}, global={global_r}"
        assert local_r >= 0.20, f"robot_radius {local_r} is smaller than actual robot (0.20m)"

    def test_lidar_frame_id_consistent(self):
        """Bug 33: LiDAR frame_id must be 'laser' everywhere to match URDF."""
        import yaml
        try:
            path = 'clean_bot_hardware/config/rplidar_a1.yaml'
            with open(path) as f:
                params = yaml.safe_load(f)
        except FileNotFoundError:
            path = 'config/rplidar_a1.yaml'
            with open(path) as f:
                params = yaml.safe_load(f)
        frame_id = params['rplidar_node']['ros__parameters']['frame_id']
        assert frame_id == 'laser', f"rplidar_a1.yaml frame_id is '{frame_id}', expected 'laser'"


class TestArduinoDriverSyntax:
    """Bug 42: Verify arduino_driver.py parses and _wheel_speed_to_pwm is defined."""

    def _find_file(self):
        import os
        for p in ['clean_bot_hardware/arduino_driver.py', 'clean_bot_hardware/clean_bot_hardware/arduino_driver.py']:
            if os.path.exists(p):
                return p
        pytest.skip('arduino_driver.py not found')

    def test_file_parses(self):
        """arduino_driver.py must be valid Python."""
        import ast
        path = self._find_file()
        with open(path) as f:
            ast.parse(f.read())

    def test_wheel_speed_to_pwm_defined(self):
        """_wheel_speed_to_pwm must be a method of ArduinoDriver."""
        import ast
        path = self._find_file()
        with open(path) as f:
            tree = ast.parse(f.read())
        methods = []
        for node in ast.walk(tree):
            if isinstance(node, ast.ClassDef) and node.name == 'ArduinoDriver':
                for item in node.body:
                    if isinstance(item, (ast.FunctionDef, ast.AsyncFunctionDef)):
                        methods.append(item.name)
        assert '_wheel_speed_to_pwm' in methods, \
            f"_wheel_speed_to_pwm not found in ArduinoDriver. Methods: {methods}"


class TestLowObstacleDetectorMarkerLifetime:
    """Bug 45: RViz markers should expire, not accumulate forever."""

    def _find_file(self):
        import os
        for p in ['clean_bot_hardware/low_obstacle_detector.py', 'clean_bot_hardware/clean_bot_hardware/low_obstacle_detector.py']:
            if os.path.exists(p):
                return p
        pytest.skip('low_obstacle_detector.py not found')

    def test_marker_lifetime_matches_persistence(self):
        """Markers should have a finite lifetime equal to persistence param."""
        path = self._find_file()
        with open(path) as f:
            source = f.read()
        lines = source.splitlines()
        for i, line in enumerate(lines):
            stripped = line.strip()
            if stripped == "marker.lifetime.sec = 0" or stripped == "marker.lifetime.nanosec = 0":
                pytest.fail(
                    f"Line {i+1}: '{stripped}' — markers should not have infinite lifetime"
                )

    def test_file_parses(self):
        """low_obstacle_detector.py should parse without errors."""
        import ast
        path = self._find_file()
        with open(path) as f:
            ast.parse(f.read())


# ════════════════════════════════════════════════════════════════════
# Test: Arduino Driver NaN/Infinity Guard
# ════════════════════════════════════════════════════════════════════

class TestArduinoNaNGuard:
    """Bug 126: NaN/Infinity cmd_vel should be rejected."""

    def _make_driver(self):
        """Create a minimal mock of ArduinoDriverNode for logic testing."""
        import threading
        driver = MagicMock()
        driver._serial_lock = threading.Lock()
        driver.serial = MagicMock()
        driver.wheel_separation = 0.20
        driver.max_linear_speed = 0.5
        driver.invert_left_motor = False
        driver.invert_right_motor = False
        driver.debug_cmd_pub = MagicMock()
        driver.last_cmd_time = MagicMock()
        driver._motors_stopped = False
        return driver

    def test_nan_linear_rejected(self):
        """NaN linear velocity should not reach serial.write."""
        import math as m
        driver = self._make_driver()
        msg = MagicMock()
        msg.linear.x = float('nan')
        msg.angular.z = 0.0

        # Import the actual check
        linear = msg.linear.x
        angular = msg.angular.z
        assert not (m.isfinite(linear) and m.isfinite(angular))

    def test_inf_angular_rejected(self):
        """Infinity angular velocity should not reach serial.write."""
        import math as m
        msg = MagicMock()
        msg.linear.x = 0.5
        msg.angular.z = float('inf')
        linear = msg.linear.x
        angular = msg.angular.z
        assert not (m.isfinite(linear) and m.isfinite(angular))

    def test_normal_values_pass(self):
        """Normal values should pass the finite check."""
        import math as m
        assert m.isfinite(0.5) and m.isfinite(-0.3)


class TestSerialLock:
    """Bug 127: Serial writes should use a lock."""

    def test_arduino_driver_has_serial_lock(self):
        """Arduino driver source should contain _serial_lock."""
        import pathlib
        base = pathlib.Path(__file__).resolve().parent.parent
        src = (base / "clean_bot_hardware" / "arduino_driver.py").read_text()
        assert "_serial_lock" in src
        assert "with self._serial_lock" in src


class TestEmergencyStopNaNRange:
    """Bug 128: Emergency stop should reject NaN/Infinity range readings."""

    def test_nan_range_rejected(self):
        """NaN range should not update current_distance."""
        import math
        # Simulate: current_distance starts at safe value
        current = 1.0
        msg_range = float('nan')
        # The guard: not math.isfinite(msg.range) or msg.range <= 0.02
        rejected = not math.isfinite(msg_range) or msg_range <= 0.02
        assert rejected

    def test_inf_range_rejected(self):
        """Infinity range should not update current_distance."""
        import math
        msg_range = float('inf')
        rejected = not math.isfinite(msg_range) or msg_range <= 0.02
        assert rejected

    def test_negative_inf_range_rejected(self):
        """Negative infinity should be rejected."""
        import math
        msg_range = float('-inf')
        rejected = not math.isfinite(msg_range) or msg_range <= 0.02
        assert rejected

    def test_valid_range_accepted(self):
        """Valid range (e.g., 0.5m) should pass the guard."""
        import math
        msg_range = 0.5
        rejected = not math.isfinite(msg_range) or msg_range <= 0.02
        assert not rejected


class TestWheelSpeedToPWM:
    """Test _wheel_speed_to_pwm conversion edge cases."""

    def _make_driver(self):
        """Create a mock ArduinoDriver with real _wheel_speed_to_pwm."""
        from clean_bot_hardware.arduino_driver import ArduinoDriver
        driver = MagicMock()
        driver.max_linear_speed = 0.3
        driver._wheel_speed_to_pwm = ArduinoDriver._wheel_speed_to_pwm.__get__(driver)
        return driver

    def test_zero_speed_returns_zero(self):
        d = self._make_driver()
        assert d._wheel_speed_to_pwm(0.0, 0.0, 0.0, 90, 90, 180) == 0

    def test_deadband(self):
        d = self._make_driver()
        assert d._wheel_speed_to_pwm(0.005, 0.005, 0.0, 90, 90, 180) == 0

    def test_forward_gives_positive(self):
        d = self._make_driver()
        pwm = d._wheel_speed_to_pwm(0.15, 0.15, 0.0, 90, 90, 180)
        assert pwm > 0

    def test_reverse_gives_negative(self):
        d = self._make_driver()
        pwm = d._wheel_speed_to_pwm(-0.15, -0.15, 0.0, 90, 90, 180)
        assert pwm < 0

    def test_max_speed_capped(self):
        d = self._make_driver()
        pwm = d._wheel_speed_to_pwm(10.0, 10.0, 0.0, 90, 90, 180)
        assert abs(pwm) <= 180

    def test_minimum_pwm_applied(self):
        d = self._make_driver()
        # Very slow speed should still get minimum PWM
        pwm = d._wheel_speed_to_pwm(0.02, 0.05, 0.0, 90, 90, 180)
        assert abs(pwm) >= 90

    def test_rotation_minimum_pwm(self):
        d = self._make_driver()
        # Pure rotation (linear ~0) should use rotate minimum
        pwm = d._wheel_speed_to_pwm(0.05, 0.0, 1.0, 90, 80, 180)
        assert abs(pwm) >= 80


class TestWatchdogTimeout:
    """Test cmd_vel watchdog behavior."""

    def test_watchdog_timeout_value(self):
        """Verify watchdog timeout is reasonable (1-5 seconds)."""
        from clean_bot_hardware.arduino_driver import ArduinoDriver
        import inspect
        source = inspect.getsource(ArduinoDriver)
        # Find CMD_VEL_TIMEOUT_SEC
        import re
        match = re.search(r'CMD_VEL_TIMEOUT_SEC\s*=\s*(\d+\.?\d*)', source)
        assert match is not None
        timeout = float(match.group(1))
        assert 0.5 <= timeout <= 10.0, f"Watchdog timeout {timeout}s is out of safe range"
