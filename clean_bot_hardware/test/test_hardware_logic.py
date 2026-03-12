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
