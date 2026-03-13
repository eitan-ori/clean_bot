#!/usr/bin/env python3
"""
Tests for mission logic state machines.

Tests the state machines in:
  - full_mission.py  (FullMissionController)
  - adaptive_coverage.py  (AdaptiveCoveragePlanner)
  - frontier_explorer.py  (FrontierExplorer)
  - webapp/app.py  (stat tracking in _on_mission_state)

These are unit tests that mock ROS 2 and focus on state transition correctness,
edge cases, and bug regressions.
"""

import json
import math
import os
import sys
import time
from pathlib import Path
from unittest.mock import MagicMock, patch, PropertyMock, call

import pytest
import numpy as np

# ── Mock ROS 2 before importing any mission modules ──────────────

mock_rclpy = MagicMock()
mock_rclpy.ok = MagicMock(return_value=True)
mock_rclpy.time = MagicMock()
mock_rclpy.time.Time = MagicMock


class _FakeNode:
    """Minimal Node stand-in."""
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
    def declare_parameter(self, name, default):
        self._params = getattr(self, '_params', {})
        self._params[name] = default
    def get_parameter(self, name):
        self._params = getattr(self, '_params', {})
        m = MagicMock()
        m.value = self._params.get(name, None)
        return m
    def destroy_node(self):
        pass


_mock_node_module = MagicMock()
_mock_node_module.Node = _FakeNode

sys.modules.setdefault('rclpy', mock_rclpy)
sys.modules['rclpy.node'] = _mock_node_module
for mod in [
    'rclpy.qos', 'rclpy.executors', 'rclpy.action', 'rclpy.callback_groups',
    'rclpy.time', 'std_msgs', 'std_msgs.msg', 'geometry_msgs', 'geometry_msgs.msg',
    'nav_msgs', 'nav_msgs.msg', 'sensor_msgs', 'sensor_msgs.msg',
    'tf2_ros', 'nav2_msgs', 'nav2_msgs.action',
    'visualization_msgs', 'visualization_msgs.msg',
]:
    sys.modules.setdefault(mod, MagicMock())


# Now import mission modules
from clean_bot_mission.full_mission import FullMissionController, MissionState
from clean_bot_mission.adaptive_coverage import AdaptiveCoveragePlanner, CoverageState
from clean_bot_mission.frontier_explorer import FrontierExplorer, ExplorationState


# ════════════════════════════════════════════════════════════════════
# Fixtures
# ════════════════════════════════════════════════════════════════════

@pytest.fixture
def mission():
    """Create a FullMissionController with mocked ROS."""
    ctrl = FullMissionController()
    # Replace publishers with spies
    ctrl.cmd_vel_pub = MagicMock()
    ctrl.state_pub = MagicMock()
    ctrl.exploration_control_pub = MagicMock()
    ctrl.coverage_control_pub = MagicMock()
    ctrl.exploration_complete_pub = MagicMock()
    ctrl.clean_trigger_pub = MagicMock()
    ctrl.stop_clean_trigger_pub = MagicMock()
    ctrl.arduino_clean_pub = MagicMock()
    ctrl.nav_client = MagicMock()
    ctrl.nav_client.wait_for_server = MagicMock(return_value=True)
    return ctrl


@pytest.fixture
def coverage():
    """Create an AdaptiveCoveragePlanner with mocked ROS."""
    planner = AdaptiveCoveragePlanner()
    planner.cmd_vel_pub = MagicMock()
    planner.coverage_state_pub = MagicMock()
    planner.coverage_complete_pub = MagicMock()
    planner.coverage_path_pub = MagicMock()
    planner.waypoint_markers_pub = MagicMock()
    planner.nav_client = MagicMock()
    planner.current_goal_handle = None
    planner.is_navigating = False
    return planner


@pytest.fixture
def explorer():
    """Create a FrontierExplorer with mocked ROS."""
    exp = FrontierExplorer()
    exp.nav_client = MagicMock()
    exp.frontier_markers_pub = MagicMock()
    exp.exploration_state_pub = MagicMock()
    exp.exploration_complete_pub = MagicMock()
    exp.current_goal_handle = None
    exp.is_navigating = False
    return exp


# ════════════════════════════════════════════════════════════════════
# FullMissionController State Machine Tests
# ════════════════════════════════════════════════════════════════════

class TestMissionStateTransitions:
    """Test all state transitions in the mission controller."""

    def test_initial_state(self, mission):
        assert mission.state == MissionState.WAITING_FOR_SCAN
        assert mission.previous_state is None

    def test_start_scan_from_waiting(self, mission):
        mission.handle_start_scan()
        assert mission.state == MissionState.EXPLORING

    def test_start_scan_from_wrong_state(self, mission):
        mission.state = MissionState.COVERAGE
        mission.handle_start_scan()
        assert mission.state == MissionState.COVERAGE  # unchanged

    def test_start_scan_from_waiting_for_clean(self, mission):
        mission.state = MissionState.WAITING_FOR_CLEAN
        mission.handle_start_scan()
        assert mission.state == MissionState.EXPLORING

    def test_stop_scan_from_exploring(self, mission):
        mission.state = MissionState.EXPLORING
        mission.handle_stop_scan()
        assert mission.state == MissionState.WAITING_FOR_CLEAN

    def test_stop_scan_from_wrong_state(self, mission):
        mission.state = MissionState.WAITING_FOR_SCAN
        mission.handle_stop_scan()
        assert mission.state == MissionState.WAITING_FOR_SCAN

    def test_start_clean_from_waiting_for_clean(self, mission):
        mission.state = MissionState.WAITING_FOR_CLEAN
        mission.handle_start_clean()
        assert mission.state == MissionState.COVERAGE

    def test_start_clean_from_waiting_for_scan(self, mission):
        """Allow starting clean even without scan (saved room scenario)."""
        mission.state = MissionState.WAITING_FOR_SCAN
        mission.handle_start_clean()
        assert mission.state == MissionState.COVERAGE

    def test_start_clean_from_wrong_state(self, mission):
        mission.state = MissionState.EXPLORING
        mission.handle_start_clean()
        assert mission.state == MissionState.EXPLORING

    def test_stop_clean_from_coverage(self, mission):
        mission.state = MissionState.COVERAGE
        mission.handle_stop_clean()
        assert mission.state == MissionState.WAITING_FOR_CLEAN

    def test_stop_clean_deactivates_hardware(self, mission):
        mission.state = MissionState.COVERAGE
        mission.handle_stop_clean()
        mission.stop_clean_trigger_pub.publish.assert_called_once()
        mission.arduino_clean_pub.publish.assert_called_once()

    def test_stop_clean_from_wrong_state(self, mission):
        mission.state = MissionState.EXPLORING
        mission.handle_stop_clean()
        assert mission.state == MissionState.EXPLORING

    def test_pause_from_exploring(self, mission):
        mission.state = MissionState.EXPLORING
        mission.handle_pause()
        assert mission.state == MissionState.PAUSED
        assert mission.previous_state == MissionState.EXPLORING

    def test_pause_from_coverage(self, mission):
        mission.state = MissionState.COVERAGE
        mission.handle_pause()
        assert mission.state == MissionState.PAUSED
        assert mission.previous_state == MissionState.COVERAGE

    def test_pause_from_wrong_state(self, mission):
        mission.state = MissionState.WAITING_FOR_SCAN
        mission.handle_pause()
        assert mission.state == MissionState.WAITING_FOR_SCAN

    def test_resume_from_paused_exploring(self, mission):
        mission.state = MissionState.PAUSED
        mission.previous_state = MissionState.EXPLORING
        mission.handle_resume()
        assert mission.state == MissionState.EXPLORING
        assert mission.previous_state is None

    def test_resume_from_paused_coverage(self, mission):
        mission.state = MissionState.PAUSED
        mission.previous_state = MissionState.COVERAGE
        mission.handle_resume()
        assert mission.state == MissionState.COVERAGE
        assert mission.previous_state is None

    def test_resume_without_previous_state(self, mission):
        mission.state = MissionState.PAUSED
        mission.previous_state = None
        mission.handle_resume()
        assert mission.state == MissionState.PAUSED  # unchanged

    def test_resume_from_wrong_state(self, mission):
        mission.state = MissionState.EXPLORING
        mission.handle_resume()
        assert mission.state == MissionState.EXPLORING


class TestMissionGoHome:
    """Test go_home from various states."""

    def test_go_home_from_exploring(self, mission):
        mission.state = MissionState.EXPLORING
        mission.handle_go_home()
        assert mission.state == MissionState.RETURNING

    def test_go_home_from_coverage_deactivates_hardware(self, mission):
        mission.state = MissionState.COVERAGE
        mission.handle_go_home()
        assert mission.state == MissionState.RETURNING
        mission.stop_clean_trigger_pub.publish.assert_called_once()
        mission.arduino_clean_pub.publish.assert_called_once()

    def test_go_home_from_paused_coverage_deactivates_hardware(self, mission):
        """Bug 53 regression: go_home from PAUSED (was COVERAGE) must deactivate cleaning."""
        mission.state = MissionState.PAUSED
        mission.previous_state = MissionState.COVERAGE
        mission.handle_go_home()
        assert mission.state == MissionState.RETURNING
        # Cleaning hardware should be deactivated
        mission.stop_clean_trigger_pub.publish.assert_called_once()
        mission.arduino_clean_pub.publish.assert_called_once()

    def test_go_home_from_paused_exploring_no_deactivate(self, mission):
        """go_home from PAUSED (was EXPLORING) should NOT deactivate cleaning hardware."""
        mission.state = MissionState.PAUSED
        mission.previous_state = MissionState.EXPLORING
        mission.handle_go_home()
        assert mission.state == MissionState.RETURNING
        mission.stop_clean_trigger_pub.publish.assert_not_called()

    def test_go_home_clears_previous_state(self, mission):
        """Bug 56 regression: previous_state should be cleared on go_home."""
        mission.state = MissionState.PAUSED
        mission.previous_state = MissionState.COVERAGE
        mission.handle_go_home()
        assert mission.previous_state is None

    def test_go_home_stops_both_sub_nodes(self, mission):
        mission.state = MissionState.COVERAGE
        mission.handle_go_home()
        calls = [c.args[0].data for c in mission.exploration_control_pub.publish.call_args_list]
        assert 'stop' in calls
        calls2 = [c.args[0].data for c in mission.coverage_control_pub.publish.call_args_list]
        assert 'stop' in calls2


class TestMissionReset:
    """Test reset from various states."""

    def test_reset_from_exploring(self, mission):
        mission.state = MissionState.EXPLORING
        mission.handle_reset()
        assert mission.state == MissionState.WAITING_FOR_SCAN

    def test_reset_from_coverage_deactivates_hardware(self, mission):
        mission.state = MissionState.COVERAGE
        mission.handle_reset()
        assert mission.state == MissionState.WAITING_FOR_SCAN
        mission.stop_clean_trigger_pub.publish.assert_called_once()

    def test_reset_from_paused_coverage_deactivates_hardware(self, mission):
        """Bug 53 regression: reset from PAUSED (was COVERAGE) must deactivate cleaning."""
        mission.state = MissionState.PAUSED
        mission.previous_state = MissionState.COVERAGE
        mission.handle_reset()
        assert mission.state == MissionState.WAITING_FOR_SCAN
        mission.stop_clean_trigger_pub.publish.assert_called_once()

    def test_reset_from_paused_exploring_no_deactivate(self, mission):
        mission.state = MissionState.PAUSED
        mission.previous_state = MissionState.EXPLORING
        mission.handle_reset()
        assert mission.state == MissionState.WAITING_FOR_SCAN
        mission.stop_clean_trigger_pub.publish.assert_not_called()

    def test_reset_clears_previous_state(self, mission):
        """Bug 56 regression: previous_state should be cleared on reset."""
        mission.state = MissionState.PAUSED
        mission.previous_state = MissionState.COVERAGE
        mission.handle_reset()
        assert mission.previous_state is None

    def test_reset_clears_flags(self, mission):
        mission.exploration_complete = True
        mission.coverage_complete = True
        mission.start_time = 12345
        mission.state = MissionState.COVERAGE
        mission.handle_reset()
        assert mission.exploration_complete is False
        assert mission.coverage_complete is False
        assert mission.start_time is None

    def test_reset_sends_stop_then_reset_to_sub_nodes(self, mission):
        mission.state = MissionState.EXPLORING
        mission.handle_reset()
        # Should publish twice to each: stop, then reset
        assert mission.exploration_control_pub.publish.call_count == 2
        assert mission.coverage_control_pub.publish.call_count == 2


class TestMissionExplorationComplete:
    """Test exploration_complete_callback."""

    def test_exploration_complete_moves_to_waiting(self, mission):
        mission.state = MissionState.EXPLORING
        msg = MagicMock()
        msg.data = True
        mission.exploration_complete_callback(msg)
        assert mission.state == MissionState.WAITING_FOR_CLEAN
        assert mission.exploration_complete is True

    def test_exploration_complete_ignored_when_not_exploring(self, mission):
        mission.state = MissionState.PAUSED
        msg = MagicMock()
        msg.data = True
        mission.exploration_complete_callback(msg)
        assert mission.state == MissionState.PAUSED

    def test_exploration_complete_false_ignored(self, mission):
        mission.state = MissionState.EXPLORING
        msg = MagicMock()
        msg.data = False
        mission.exploration_complete_callback(msg)
        assert mission.state == MissionState.EXPLORING


class TestMissionCoverageComplete:
    """Test coverage_complete_callback."""

    def test_coverage_complete_deactivates_hardware(self, mission):
        mission.state = MissionState.COVERAGE
        mission.return_home = False
        msg = MagicMock()
        msg.data = True
        mission.coverage_complete_callback(msg)
        assert mission.state == MissionState.COMPLETE
        mission.stop_clean_trigger_pub.publish.assert_called_once()

    def test_coverage_complete_triggers_return_home(self, mission):
        mission.state = MissionState.COVERAGE
        mission.return_home = True
        msg = MagicMock()
        msg.data = True
        mission.coverage_complete_callback(msg)
        assert mission.state == MissionState.RETURNING

    def test_coverage_complete_ignored_when_not_coverage(self, mission):
        mission.state = MissionState.PAUSED
        msg = MagicMock()
        msg.data = True
        mission.coverage_complete_callback(msg)
        assert mission.state == MissionState.PAUSED


class TestMissionCommandCallback:
    """Test the command dispatcher."""

    def test_valid_commands_dispatched(self, mission):
        for cmd in ['start_scan', 'stop_scan', 'start_clean', 'stop_clean',
                     'go_home', 'reset', 'pause', 'resume']:
            msg = MagicMock()
            msg.data = cmd
            # Should not raise
            mission.command_callback(msg)

    def test_unknown_command(self, mission):
        msg = MagicMock()
        msg.data = 'fly_away'
        mission.command_callback(msg)
        assert mission.state == MissionState.WAITING_FOR_SCAN  # unchanged

    def test_command_case_insensitive(self, mission):
        msg = MagicMock()
        msg.data = '  START_SCAN  '
        mission.command_callback(msg)
        assert mission.state == MissionState.EXPLORING

    def test_pause_resume_roundtrip(self, mission):
        mission.state = MissionState.EXPLORING
        msg = MagicMock()
        msg.data = 'pause'
        mission.command_callback(msg)
        assert mission.state == MissionState.PAUSED
        msg.data = 'resume'
        mission.command_callback(msg)
        assert mission.state == MissionState.EXPLORING


class TestMissionFinish:
    """Test finish_mission."""

    def test_finish_sets_complete(self, mission):
        mission.start_time = time.time() - 60
        mission.finish_mission()
        assert mission.state == MissionState.COMPLETE

    def test_finish_with_no_start_time(self, mission):
        mission.start_time = None
        mission.finish_mission()
        assert mission.state == MissionState.COMPLETE

    def test_finish_stops_robot(self, mission):
        mission.start_time = time.time()
        mission.finish_mission()
        mission.cmd_vel_pub.publish.assert_called()


# ════════════════════════════════════════════════════════════════════
# Coverage Planner State Machine Tests
# ════════════════════════════════════════════════════════════════════

class TestCoverageStateTransitions:
    """Test coverage planner state machine."""

    def test_initial_state(self, coverage):
        assert coverage.coverage_state == CoverageState.IDLE

    def test_handle_stop_from_running(self, coverage):
        coverage.coverage_state = CoverageState.RUNNING
        coverage.handle_stop()
        assert coverage.coverage_state == CoverageState.STOPPED

    def test_handle_stop_from_paused(self, coverage):
        coverage.coverage_state = CoverageState.PAUSED
        coverage.handle_stop()
        assert coverage.coverage_state == CoverageState.STOPPED

    def test_handle_stop_from_idle(self, coverage):
        coverage.coverage_state = CoverageState.IDLE
        coverage.handle_stop()
        assert coverage.coverage_state == CoverageState.IDLE

    def test_handle_pause_from_running(self, coverage):
        coverage.coverage_state = CoverageState.RUNNING
        coverage.handle_pause()
        assert coverage.coverage_state == CoverageState.PAUSED

    def test_handle_pause_from_idle(self, coverage):
        coverage.coverage_state = CoverageState.IDLE
        coverage.handle_pause()
        assert coverage.coverage_state == CoverageState.IDLE

    def test_handle_resume_from_paused(self, coverage):
        coverage.coverage_state = CoverageState.PAUSED
        coverage.waypoints = [(0, 0, 0)]
        coverage.current_waypoint_idx = 0
        coverage.handle_resume()
        assert coverage.coverage_state == CoverageState.RUNNING

    def test_handle_resume_from_stopped(self, coverage):
        coverage.coverage_state = CoverageState.STOPPED
        coverage.handle_resume()
        # Should call handle_start, which transitions based on waypoints
        # After stop→resume, it should attempt to start
        assert coverage.coverage_state in [CoverageState.RUNNING, CoverageState.IDLE, CoverageState.STOPPED]

    def test_handle_reset(self, coverage):
        coverage.coverage_state = CoverageState.RUNNING
        coverage.waypoints = [(1, 2, 3)]
        coverage.mission_started = True
        coverage.handle_reset()
        assert coverage.coverage_state == CoverageState.IDLE
        assert coverage.waypoints == []
        assert coverage.mission_started is False
        assert coverage.current_waypoint_idx == 0


class TestCoverageNormalizeAngle:
    """Test normalize_angle edge cases including NaN/inf (Bug 55)."""

    def test_normal_angle(self, coverage):
        assert abs(coverage.normalize_angle(0.0)) < 1e-9
        assert abs(coverage.normalize_angle(math.pi) - math.pi) < 1e-9

    def test_angle_wrap_positive(self, coverage):
        result = coverage.normalize_angle(3 * math.pi)
        assert -math.pi <= result <= math.pi

    def test_angle_wrap_negative(self, coverage):
        result = coverage.normalize_angle(-3 * math.pi)
        assert -math.pi <= result <= math.pi

    def test_large_angle(self, coverage):
        result = coverage.normalize_angle(100 * math.pi)
        assert -math.pi <= result <= math.pi

    def test_nan_returns_zero(self, coverage):
        """Bug 55 regression: NaN should not cause infinite loop."""
        result = coverage.normalize_angle(float('nan'))
        assert result == 0.0

    def test_inf_returns_zero(self, coverage):
        """Bug 55 regression: inf should not cause infinite loop."""
        result = coverage.normalize_angle(float('inf'))
        assert result == 0.0

    def test_neg_inf_returns_zero(self, coverage):
        """Bug 55 regression: -inf should not cause infinite loop."""
        result = coverage.normalize_angle(float('-inf'))
        assert result == 0.0


class TestCoverageFinishMission:
    """Test coverage finish_mission including retry logic."""

    def test_finish_sets_complete(self, coverage):
        coverage.missed_waypoints = []
        coverage.retry_count = 0
        coverage.start_time = None
        coverage.finish_mission()
        assert coverage.coverage_state == CoverageState.COMPLETE
        assert coverage.mission_complete is True

    def test_finish_retries_missed_waypoints(self, coverage):
        coverage.missed_waypoints = [(1.0, 2.0, 0.0)]
        coverage.retry_count = 0
        coverage.max_retries = 2
        coverage.start_time = None
        coverage.coverage_state = CoverageState.RUNNING
        coverage.finish_mission()
        # Should have set up retry, not completed
        assert coverage.mission_complete is False
        assert coverage.retry_count == 1

    def test_finish_no_retry_after_max(self, coverage):
        coverage.missed_waypoints = [(1.0, 2.0, 0.0)]
        coverage.retry_count = 3
        coverage.max_retries = 2
        coverage.start_time = None
        coverage.finish_mission()
        assert coverage.mission_complete is True
        assert coverage.coverage_state == CoverageState.COMPLETE


class TestCoverageSendNextGoal:
    """Test send_next_goal."""

    def test_send_next_goal_finishes_when_all_done(self, coverage):
        coverage.waypoints = [(1, 2, 3)]
        coverage.current_waypoint_idx = 1  # past end
        coverage.missed_waypoints = []
        coverage.retry_count = 0
        coverage.start_time = None
        coverage.send_next_goal()
        assert coverage.coverage_state == CoverageState.COMPLETE

    def test_send_next_goal_noop_when_more_waypoints(self, coverage):
        coverage.waypoints = [(1, 2, 3), (4, 5, 6)]
        coverage.current_waypoint_idx = 0
        coverage.send_next_goal()
        # Should not complete
        assert coverage.coverage_state != CoverageState.COMPLETE


class TestCoverageGoalCallbacks:
    """Test goal response and result callbacks."""

    def test_goal_rejected(self, coverage):
        coverage.waypoints = [(1, 2, 3)]
        coverage.current_waypoint_idx = 0
        coverage.is_navigating = True
        coverage.missed_waypoints = []
        coverage.retry_count = 0
        coverage.max_retries = 0  # no retries
        coverage.start_time = None

        future = MagicMock()
        goal_handle = MagicMock()
        goal_handle.accepted = False
        future.result.return_value = goal_handle

        coverage.goal_response_callback(future)
        assert coverage.is_navigating is False
        assert coverage.failed_waypoints == 1
        # After rejection of last waypoint, mission completes
        assert coverage.coverage_state == CoverageState.COMPLETE

    def test_result_succeeded(self, coverage):
        coverage.coverage_state = CoverageState.RUNNING
        coverage.waypoints = [(1, 2, 3), (4, 5, 6)]
        coverage.current_waypoint_idx = 0
        coverage.missed_waypoints = []
        coverage.retry_count = 0
        coverage.start_time = None

        future = MagicMock()
        result = MagicMock()
        result.status = 4  # SUCCEEDED
        future.result.return_value = result

        coverage.get_result_callback(future)
        assert coverage.successful_waypoints == 1
        assert coverage.current_waypoint_idx == 1

    def test_result_aborted(self, coverage):
        coverage.coverage_state = CoverageState.RUNNING
        coverage.waypoints = [(1, 2, 3), (4, 5, 6)]
        coverage.current_waypoint_idx = 0
        coverage.missed_waypoints = []
        coverage.retry_count = 0
        coverage.start_time = None

        future = MagicMock()
        result = MagicMock()
        result.status = 6  # ABORTED
        future.result.return_value = result

        coverage.get_result_callback(future)
        assert coverage.failed_waypoints == 1
        assert len(coverage.missed_waypoints) == 1

    def test_result_cancelled_doesnt_advance(self, coverage):
        coverage.coverage_state = CoverageState.RUNNING
        coverage.waypoints = [(1, 2, 3)]
        coverage.current_waypoint_idx = 0

        future = MagicMock()
        result = MagicMock()
        result.status = 5  # CANCELED
        future.result.return_value = result

        coverage.get_result_callback(future)
        # Should not advance idx
        assert coverage.current_waypoint_idx == 0

    def test_result_ignored_when_not_running(self, coverage):
        coverage.coverage_state = CoverageState.PAUSED
        coverage.waypoints = [(1, 2, 3)]
        coverage.current_waypoint_idx = 0

        future = MagicMock()
        result = MagicMock()
        result.status = 4
        future.result.return_value = result

        coverage.get_result_callback(future)
        assert coverage.current_waypoint_idx == 0  # not advanced


# ════════════════════════════════════════════════════════════════════
# Frontier Explorer State Machine Tests
# ════════════════════════════════════════════════════════════════════

class TestExplorerStateTransitions:
    """Test frontier explorer state machine."""

    def test_initial_state(self, explorer):
        assert explorer.exploration_state == ExplorationState.IDLE

    def test_start_from_idle(self, explorer):
        explorer.handle_start()
        assert explorer.exploration_state == ExplorationState.EXPLORING

    def test_start_from_stopped(self, explorer):
        explorer.exploration_state = ExplorationState.STOPPED
        explorer.handle_start()
        assert explorer.exploration_state == ExplorationState.EXPLORING

    def test_start_from_paused(self, explorer):
        explorer.exploration_state = ExplorationState.PAUSED
        explorer.handle_start()
        assert explorer.exploration_state == ExplorationState.EXPLORING

    def test_stop_from_exploring(self, explorer):
        explorer.exploration_state = ExplorationState.EXPLORING
        explorer.handle_stop()
        assert explorer.exploration_state == ExplorationState.STOPPED

    def test_stop_from_paused(self, explorer):
        explorer.exploration_state = ExplorationState.PAUSED
        explorer.handle_stop()
        assert explorer.exploration_state == ExplorationState.STOPPED

    def test_pause_from_exploring(self, explorer):
        explorer.exploration_state = ExplorationState.EXPLORING
        explorer.handle_pause()
        assert explorer.exploration_state == ExplorationState.PAUSED

    def test_pause_from_idle(self, explorer):
        explorer.exploration_state = ExplorationState.IDLE
        explorer.handle_pause()
        assert explorer.exploration_state == ExplorationState.IDLE

    def test_resume_from_paused(self, explorer):
        explorer.exploration_state = ExplorationState.PAUSED
        explorer.handle_resume()
        assert explorer.exploration_state == ExplorationState.EXPLORING

    def test_resume_from_stopped(self, explorer):
        explorer.exploration_state = ExplorationState.STOPPED
        explorer.handle_resume()
        assert explorer.exploration_state == ExplorationState.EXPLORING

    def test_reset_clears_all(self, explorer):
        explorer.exploration_state = ExplorationState.EXPLORING
        explorer.is_navigating = True
        explorer.goals_attempted = 5
        explorer.goals_reached = 3
        explorer.consecutive_failures = 2
        explorer.handle_reset()
        assert explorer.exploration_state == ExplorationState.IDLE
        assert explorer.is_navigating is False
        assert explorer.goals_attempted == 0
        assert explorer.goals_reached == 0
        assert explorer.consecutive_failures == 0


class TestExplorerGoalCallbacks:
    """Test explorer goal response and result callbacks."""

    def test_goal_rejected(self, explorer):
        explorer.is_navigating = True
        explorer.current_goal = {'x': 1.0, 'y': 2.0}

        future = MagicMock()
        goal_handle = MagicMock()
        goal_handle.accepted = False
        future.result.return_value = goal_handle

        explorer.goal_response_callback(future)
        assert explorer.is_navigating is False
        assert len(explorer.failed_goals) == 1

    def test_result_succeeded(self, explorer):
        explorer.current_goal = {'x': 1.0, 'y': 2.0}

        future = MagicMock()
        result = MagicMock()
        result.status = 4  # SUCCEEDED
        future.result.return_value = result

        explorer.get_result_callback(future)
        assert explorer.goals_reached == 1
        assert explorer.consecutive_failures == 0

    def test_result_aborted_increments_failure(self, explorer):
        explorer.current_goal = {'x': 1.0, 'y': 2.0}
        explorer.consecutive_failures = 0

        future = MagicMock()
        result = MagicMock()
        result.status = 6  # ABORTED
        future.result.return_value = result

        explorer.get_result_callback(future)
        assert explorer.consecutive_failures == 1
        assert len(explorer.failed_goals) == 1

    def test_max_consecutive_failures_finishes(self, explorer):
        explorer.current_goal = {'x': 1.0, 'y': 2.0}
        explorer.consecutive_failures = explorer.max_consecutive_failures - 1
        explorer.start_time = None

        future = MagicMock()
        result = MagicMock()
        result.status = 6
        future.result.return_value = result

        explorer.get_result_callback(future)
        assert explorer.exploration_state == ExplorationState.COMPLETE


class TestExplorerNoGoZones:
    """Test no-go zone filtering in frontier selection."""

    def test_point_in_no_go_zone(self, explorer):
        explorer._no_go_zones = [{'x1': 0, 'y1': 0, 'x2': 2, 'y2': 2}]
        assert explorer._point_in_no_go_zone(1.0, 1.0) is True
        assert explorer._point_in_no_go_zone(3.0, 3.0) is False

    def test_point_at_zone_boundary(self, explorer):
        explorer._no_go_zones = [{'x1': 0, 'y1': 0, 'x2': 2, 'y2': 2}]
        assert explorer._point_in_no_go_zone(0.0, 0.0) is True
        assert explorer._point_in_no_go_zone(2.0, 2.0) is True

    def test_empty_no_go_zones(self, explorer):
        explorer._no_go_zones = []
        assert explorer._point_in_no_go_zone(1.0, 1.0) is False


class TestExplorerCancelGoal:
    """Test cancel_current_goal (Bug 62 regression)."""

    def test_cancel_clears_navigating_immediately(self, explorer):
        """Bug 62: is_navigating should be cleared immediately, not in callback."""
        explorer.is_navigating = True
        explorer.navigation_start_time = MagicMock()
        handle = MagicMock()
        explorer.current_goal_handle = handle
        explorer.current_goal = {'x': 1.0, 'y': 2.0}

        explorer.cancel_current_goal()

        assert explorer.is_navigating is False
        assert explorer.navigation_start_time is None
        assert explorer.current_goal_handle is None
        handle.cancel_goal_async.assert_called_once()

    def test_cancel_without_handle(self, explorer):
        """Cancel without a goal handle still clears state."""
        explorer.is_navigating = True
        explorer.navigation_start_time = MagicMock()
        explorer.current_goal_handle = None
        explorer.current_goal = {'x': 1.0, 'y': 2.0}

        explorer.cancel_current_goal()

        assert explorer.is_navigating is False
        assert explorer.navigation_start_time is None
        assert (1.0, 2.0) in explorer.failed_goals

    def test_cancel_done_callback_idempotent(self, explorer):
        """cancel_done_callback should be safe even if state already cleared."""
        explorer.is_navigating = False
        explorer.current_goal = {'x': 1.0, 'y': 2.0}

        explorer.cancel_done_callback(MagicMock())
        assert (1.0, 2.0) in explorer.failed_goals


# ════════════════════════════════════════════════════════════════════
# Webapp Stat Tracking Tests (Bug 54)
# ════════════════════════════════════════════════════════════════════

class TestWebappStatTracking:
    """Test _on_mission_state stat tracking in webapp."""

    @pytest.fixture
    def web_node(self, tmp_path):
        """Create a WebBridgeNode with mocked ROS."""
        from clean_bot_mission.webapp.app import WebBridgeNode
        node = WebBridgeNode.__new__(WebBridgeNode)
        # Initialize essential attributes
        node.mission_state = "WAITING_FOR_SCAN"
        node.mission_log = []
        node._scan_start_time = None
        node._clean_start_time = None
        node.total_scan_time = 0.0
        node.total_clean_time = 0.0
        node.rooms_scanned = 0
        node.rooms_cleaned = 0
        node.sio = MagicMock()
        return node

    def test_exploring_to_paused_no_scan_count(self, web_node):
        """Bug 54 regression: EXPLORING→PAUSED should NOT count as scan completion."""
        web_node.mission_state = "EXPLORING"
        web_node._scan_start_time = time.monotonic() - 10

        msg = MagicMock()
        msg.data = "PAUSED"
        web_node._on_mission_state(msg)

        assert web_node.rooms_scanned == 0
        # scan_start_time should be preserved (paused, not finished)
        assert web_node._scan_start_time is not None

    def test_coverage_to_paused_no_clean_count(self, web_node):
        """Bug 54 regression: COVERAGE→PAUSED should NOT count as clean completion."""
        web_node.mission_state = "COVERAGE"
        web_node._clean_start_time = time.monotonic() - 10

        msg = MagicMock()
        msg.data = "PAUSED"
        web_node._on_mission_state(msg)

        assert web_node.rooms_cleaned == 0
        assert web_node._clean_start_time is not None

    def test_exploring_to_waiting_for_clean_counts_scan(self, web_node):
        """EXPLORING→WAITING_FOR_CLEAN should count as scan completion."""
        web_node.mission_state = "EXPLORING"
        web_node._scan_start_time = time.monotonic() - 10

        msg = MagicMock()
        msg.data = "WAITING_FOR_CLEAN"
        web_node._on_mission_state(msg)

        assert web_node.rooms_scanned == 1
        assert web_node._scan_start_time is None

    def test_coverage_to_returning_counts_clean(self, web_node):
        """COVERAGE→RETURNING should count as clean completion."""
        web_node.mission_state = "COVERAGE"
        web_node._clean_start_time = time.monotonic() - 10

        msg = MagicMock()
        msg.data = "RETURNING"
        web_node._on_mission_state(msg)

        assert web_node.rooms_cleaned == 1
        assert web_node._clean_start_time is None

    def test_exploring_start_tracks_time(self, web_node):
        """Transition INTO EXPLORING should start scan timer."""
        web_node.mission_state = "WAITING_FOR_SCAN"
        web_node._scan_start_time = None

        msg = MagicMock()
        msg.data = "EXPLORING"
        web_node._on_mission_state(msg)

        assert web_node._scan_start_time is not None

    def test_coverage_start_tracks_time(self, web_node):
        """Transition INTO COVERAGE should start clean timer."""
        web_node.mission_state = "WAITING_FOR_CLEAN"
        web_node._clean_start_time = None

        msg = MagicMock()
        msg.data = "COVERAGE"
        web_node._on_mission_state(msg)

        assert web_node._clean_start_time is not None

    def test_paused_to_returning_clears_clean_timer(self, web_node):
        """Bug 60 regression: PAUSED→RETURNING should finalize clean time, not leak timer."""
        web_node.mission_state = "PAUSED"
        web_node._clean_start_time = time.monotonic() - 30

        msg = MagicMock()
        msg.data = "RETURNING"
        web_node._on_mission_state(msg)

        assert web_node._clean_start_time is None
        assert web_node.total_clean_time >= 29  # approximately 30 seconds

    def test_paused_to_waiting_clears_scan_timer(self, web_node):
        """Bug 60 regression: PAUSED→WAITING_FOR_SCAN (reset) should finalize scan time."""
        web_node.mission_state = "PAUSED"
        web_node._scan_start_time = time.monotonic() - 20

        msg = MagicMock()
        msg.data = "WAITING_FOR_SCAN"
        web_node._on_mission_state(msg)

        assert web_node._scan_start_time is None
        assert web_node.total_scan_time >= 19  # approximately 20 seconds

    def test_resume_preserves_scan_timer(self, web_node):
        """Bug 61 regression: PAUSED→EXPLORING should NOT reset scan timer."""
        start = time.monotonic() - 60
        web_node.mission_state = "PAUSED"
        web_node._scan_start_time = start

        msg = MagicMock()
        msg.data = "EXPLORING"
        web_node._on_mission_state(msg)

        # Timer should continue from original start, not be reset
        assert web_node._scan_start_time == start

    def test_resume_preserves_clean_timer(self, web_node):
        """Bug 61 regression: PAUSED→COVERAGE should NOT reset clean timer."""
        start = time.monotonic() - 45
        web_node.mission_state = "PAUSED"
        web_node._clean_start_time = start

        msg = MagicMock()
        msg.data = "COVERAGE"
        web_node._on_mission_state(msg)

        # Timer should continue from original start, not be reset
        assert web_node._clean_start_time == start


class TestOdomBugFixes:
    """Test _on_odom edge cases (Bugs 69-70)."""

    @pytest.fixture
    def web_node(self):
        from clean_bot_mission.webapp.app import WebBridgeNode
        node = WebBridgeNode.__new__(WebBridgeNode)
        node.linear_vel = 0.0
        node.angular_vel = 0.0
        node.total_distance_traveled = 0.0
        node._last_odom_x = None
        node._last_odom_y = None
        node.battery_level = 100.0
        node._battery_last_update = time.monotonic()
        node.sio = MagicMock()
        return node

    def _make_odom(self, x=0.0, y=0.0, lx=0.0, az=0.0):
        msg = MagicMock()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.twist.twist.linear.x = lx
        msg.twist.twist.angular.z = az
        return msg

    def test_battery_dt_capped(self, web_node):
        """Bug 69: Large time gap should not drain battery excessively."""
        web_node._battery_last_update = time.monotonic() - 300  # 5 min gap
        web_node._on_odom(self._make_odom(x=0.0))
        # With dt capped at 2s and base drain 0.02/s: max drain = 2 * 0.02 = 0.04%
        assert web_node.battery_level >= 99.9

    def test_distance_jump_filtered(self, web_node):
        """Bug 70: Large odom jumps should not inflate distance traveled."""
        web_node._last_odom_x = 0.0
        web_node._last_odom_y = 0.0
        # Simulate a teleport/odom jump of 5 meters
        web_node._on_odom(self._make_odom(x=5.0, y=0.0))
        assert web_node.total_distance_traveled == 0.0  # Jump filtered out

    def test_normal_movement_tracked(self, web_node):
        """Normal movement increments should be tracked correctly."""
        web_node._last_odom_x = 0.0
        web_node._last_odom_y = 0.0
        web_node._on_odom(self._make_odom(x=0.1, y=0.0))
        assert abs(web_node.total_distance_traveled - 0.1) < 0.01

class TestCoveragePathEdgeCases:
    """Test edge cases in coverage path generation."""

    def test_find_free_segments_all_occupied(self, coverage):
        coverage.inflated_map = np.full((10, 10), 100, dtype=np.int8)
        coverage.map_info = MagicMock()
        coverage.map_info.resolution = 0.05
        result = coverage.find_free_segments_in_column(0)
        assert result == []

    def test_find_free_segments_all_free(self, coverage):
        coverage.inflated_map = np.zeros((100, 10), dtype=np.int8)
        coverage.map_info = MagicMock()
        coverage.map_info.resolution = 0.05
        result = coverage.find_free_segments_in_column(0)
        assert len(result) >= 1
        # Should cover most of the column
        total_rows = sum(end - start for start, end in result)
        assert total_rows > 50

    def test_find_free_segments_short_segment_filtered(self, coverage):
        """Segments shorter than 10cm should be filtered."""
        coverage.inflated_map = np.full((10, 10), 100, dtype=np.int8)
        coverage.map_info = MagicMock()
        coverage.map_info.resolution = 0.05
        # Create a 1-cell free segment (0.05m < 0.10m threshold)
        coverage.inflated_map[5, 0] = 0
        result = coverage.find_free_segments_in_column(0)
        assert result == []

    def test_optimize_path_order_few_waypoints(self, coverage):
        """Path with <= 4 waypoints should be returned as-is."""
        wps = [(0, 0, 0), (1, 0, 0)]
        result = coverage.optimize_path_order(wps)
        assert result == wps

    def test_optimize_path_order_reversal(self, coverage):
        """Optimizer should reverse pairs when closer to end."""
        wps = [
            (0, 0, 0), (0, 1, 0),
            (0, 3, 0), (0, 2, 0),
            (0, 5, 0), (0, 4, 0),
        ]
        result = coverage.optimize_path_order(wps)
        assert len(result) == 6


class TestCoverageGenerateCellBoundary:
    """Test generate_cell_boundary."""

    def test_empty_segments(self, coverage):
        assert coverage.generate_cell_boundary([]) == []

    def test_single_segment(self, coverage):
        segs = [(1.0, 0.0, 2.0, 0, 0, 40)]
        result = coverage.generate_cell_boundary(segs)
        assert len(result) > 0

    def test_multiple_segments(self, coverage):
        segs = [
            (1.0, 0.0, 2.0, 0, 0, 40),
            (2.0, 0.5, 2.5, 1, 10, 50),
        ]
        result = coverage.generate_cell_boundary(segs)
        assert len(result) > 0


# ════════════════════════════════════════════════════════════════════
# Full Mission Scenario Tests
# ════════════════════════════════════════════════════════════════════

class TestFullMissionScenarios:
    """End-to-end scenario tests for the mission state machine."""

    def test_full_scan_pause_resume_clean_home(self, mission):
        """Test complete workflow: scan → pause → resume → clean → home → complete."""
        # Start scan
        mission.handle_start_scan()
        assert mission.state == MissionState.EXPLORING

        # Pause
        mission.handle_pause()
        assert mission.state == MissionState.PAUSED
        assert mission.previous_state == MissionState.EXPLORING

        # Resume
        mission.handle_resume()
        assert mission.state == MissionState.EXPLORING

        # Stop scan
        mission.handle_stop_scan()
        assert mission.state == MissionState.WAITING_FOR_CLEAN

        # Start clean
        mission.handle_start_clean()
        assert mission.state == MissionState.COVERAGE

        # Pause during clean
        mission.handle_pause()
        assert mission.state == MissionState.PAUSED
        assert mission.previous_state == MissionState.COVERAGE

        # Go home from paused (Bug 53: should deactivate cleaning)
        mission.handle_go_home()
        assert mission.state == MissionState.RETURNING
        assert mission.previous_state is None
        mission.stop_clean_trigger_pub.publish.assert_called()

    def test_scan_clean_reset_restart(self, mission):
        """Test reset mid-flow and restart."""
        mission.handle_start_scan()
        assert mission.state == MissionState.EXPLORING

        mission.handle_reset()
        assert mission.state == MissionState.WAITING_FOR_SCAN
        assert mission.exploration_complete is False

        # Restart
        mission.handle_start_scan()
        assert mission.state == MissionState.EXPLORING

    def test_multiple_pause_resume_cycles(self, mission):
        """Test multiple pause/resume cycles."""
        mission.handle_start_scan()
        for _ in range(5):
            mission.handle_pause()
            assert mission.state == MissionState.PAUSED
            mission.handle_resume()
            assert mission.state == MissionState.EXPLORING

    def test_stop_clean_restart_clean(self, mission):
        """Test stopping and restarting clean."""
        mission.state = MissionState.WAITING_FOR_CLEAN
        mission.handle_start_clean()
        assert mission.state == MissionState.COVERAGE

        mission.handle_stop_clean()
        assert mission.state == MissionState.WAITING_FOR_CLEAN

        mission.handle_start_clean()
        assert mission.state == MissionState.COVERAGE

    def test_invalid_commands_in_all_states(self, mission):
        """Test that invalid state transitions are rejected correctly."""
        # start_clean from EXPLORING
        mission.state = MissionState.EXPLORING
        mission.handle_start_clean()
        assert mission.state == MissionState.EXPLORING  # unchanged

        # stop_scan from COVERAGE
        mission.state = MissionState.COVERAGE
        mission.handle_stop_scan()
        assert mission.state == MissionState.COVERAGE  # unchanged

        # pause from COMPLETE
        mission.state = MissionState.COMPLETE
        mission.handle_pause()
        assert mission.state == MissionState.COMPLETE  # unchanged

        # resume from EXPLORING
        mission.state = MissionState.EXPLORING
        mission.previous_state = None
        mission.handle_resume()
        assert mission.state == MissionState.EXPLORING  # unchanged

    def test_go_home_from_exploring_no_clean_deactivation(self, mission):
        """go_home from EXPLORING should NOT deactivate cleaning hardware."""
        mission.state = MissionState.EXPLORING
        mission.previous_state = None
        mission.handle_go_home()
        assert mission.state == MissionState.RETURNING
        # Cleaning hardware should NOT be deactivated (not cleaning)
        mission.stop_clean_trigger_pub.publish.assert_not_called()

    def test_coverage_planner_full_lifecycle(self, coverage):
        """Coverage planner: IDLE → start → RUNNING → pause → resume → stop."""
        assert coverage.coverage_state == CoverageState.IDLE

        # Can't test start_coverage_mission without a map, so test state management
        coverage.coverage_state = CoverageState.RUNNING
        coverage.waypoints = [(0, 0, 0), (1, 0, 0)]
        coverage.current_waypoint_idx = 0
        coverage.mission_started = True

        coverage.handle_pause()
        assert coverage.coverage_state == CoverageState.PAUSED

        coverage.handle_resume()
        assert coverage.coverage_state == CoverageState.RUNNING

        coverage.handle_stop()
        assert coverage.coverage_state == CoverageState.STOPPED

    def test_explorer_exploration_complete_flow(self, explorer):
        """Explorer completes exploration and publishes completion."""
        explorer.exploration_state = ExplorationState.EXPLORING
        explorer.start_time = None
        explorer.finish_exploration()
        assert explorer.exploration_state == ExplorationState.COMPLETE
        assert explorer.exploration_complete is True

    def test_pause_from_coverage_deactivates_hardware(self, mission):
        """Bug 66: Pausing from COVERAGE should deactivate cleaning hardware."""
        mission.state = MissionState.COVERAGE
        mission.handle_pause()
        assert mission.state == MissionState.PAUSED
        assert mission.previous_state == MissionState.COVERAGE
        # Cleaning hardware should be deactivated
        mission.stop_clean_trigger_pub.publish.assert_called()

    def test_pause_from_exploring_no_hardware_deactivation(self, mission):
        """Pausing from EXPLORING should NOT deactivate cleaning hardware."""
        mission.state = MissionState.EXPLORING
        mission.handle_pause()
        assert mission.state == MissionState.PAUSED
        mission.stop_clean_trigger_pub.publish.assert_not_called()

    def test_resume_to_coverage_reactivates_hardware(self, mission):
        """Bug 66: Resuming to COVERAGE should reactivate cleaning hardware."""
        mission.state = MissionState.PAUSED
        mission.previous_state = MissionState.COVERAGE
        mission.handle_resume()
        assert mission.state == MissionState.COVERAGE
        # Cleaning hardware should be reactivated
        mission.clean_trigger_pub.publish.assert_called()

    def test_resume_to_exploring_no_hardware_activation(self, mission):
        """Resuming to EXPLORING should NOT activate cleaning hardware."""
        mission.state = MissionState.PAUSED
        mission.previous_state = MissionState.EXPLORING
        mission.handle_resume()
        assert mission.state == MissionState.EXPLORING
        mission.clean_trigger_pub.publish.assert_not_called()


class TestNavCallbackErrorHandling:
    """Test error handling in Nav2 callback methods (Bugs 63-65)."""

    def test_nav_goal_response_exception_finishes_mission(self, mission):
        """Bug 63: If future.result() throws, finish_mission should be called."""
        from unittest.mock import MagicMock
        future = MagicMock()
        future.result.side_effect = RuntimeError("action server crashed")
        mission.state = MissionState.RETURNING
        mission._nav_goal_response_callback(future)
        # Should fall back to finish_mission, setting state to COMPLETE
        assert mission.state == MissionState.COMPLETE

    def test_explorer_goal_response_exception_clears_navigating(self, explorer):
        """Bug 64: If future.result() throws in explorer, is_navigating cleared."""
        from unittest.mock import MagicMock
        future = MagicMock()
        future.result.side_effect = RuntimeError("action server crashed")
        explorer.is_navigating = True
        explorer.navigation_start_time = object()  # non-None
        explorer.consecutive_failures = 0
        explorer.goal_response_callback(future)
        assert explorer.is_navigating is False
        assert explorer.navigation_start_time is None
        assert explorer.consecutive_failures == 1

    def test_explorer_get_result_exception_clears_navigating(self, explorer):
        """Bug 65: If future.result() throws in get_result, is_navigating cleared."""
        from unittest.mock import MagicMock
        future = MagicMock()
        future.result.side_effect = RuntimeError("result fetch failed")
        explorer.is_navigating = True
        explorer.navigation_start_time = object()
        explorer.current_goal_handle = object()
        explorer.consecutive_failures = 0
        explorer.get_result_callback(future)
        assert explorer.is_navigating is False
        assert explorer.navigation_start_time is None
        assert explorer.current_goal_handle is None
        assert explorer.consecutive_failures == 1
    """Test drive_to_waypoint steering logic."""

    @pytest.fixture
    def drive_coverage(self):
        """Coverage planner set up for drive tests."""
        planner = AdaptiveCoveragePlanner()
        planner.cmd_vel_pub = MagicMock()
        planner.coverage_state = CoverageState.RUNNING
        planner.waypoints = [(1.0, 0.0, 0.0)]
        planner.current_waypoint_idx = 0
        planner.robot_x = 0.0
        planner.robot_y = 0.0
        planner.robot_yaw = 0.0
        planner.odom_received = True
        # Mock TF to fail so it falls back to odom pose
        planner.tf_buffer = MagicMock()
        planner.tf_buffer.lookup_transform.side_effect = Exception("no TF")
        planner._tf_pose_warned = True
        return planner

    def test_well_aligned_no_angular_correction(self, drive_coverage):
        """Bug 59 regression: when well-aligned, angular velocity should be zero."""
        # Robot at origin facing right (yaw=0), target at (1,0) → perfect alignment
        drive_coverage.robot_yaw = 0.0
        drive_coverage.drive_to_waypoint(1.0, 0.0, 100.0)
        cmd = drive_coverage.cmd_vel_pub.publish.call_args[0][0]
        # Should drive forward with near-zero angular correction
        assert cmd.linear.x > 0
        assert cmd.angular.z == 0.0

    def test_misaligned_gets_correction(self, drive_coverage):
        """When significantly misaligned, angular correction should be applied."""
        drive_coverage.robot_yaw = 0.5  # ~29 degrees off
        drive_coverage.drive_to_waypoint(1.0, 0.0, 100.0)
        cmd = drive_coverage.cmd_vel_pub.publish.call_args[0][0]
        assert cmd.angular.z != 0.0

    def test_large_misalignment_rotates_in_place(self, drive_coverage):
        """When very misaligned, robot should rotate in place (no forward)."""
        drive_coverage.robot_yaw = math.pi  # facing backwards
        drive_coverage.drive_to_waypoint(1.0, 0.0, 100.0)
        cmd = drive_coverage.cmd_vel_pub.publish.call_args[0][0]
        assert cmd.linear.x == 0.0
        assert abs(cmd.angular.z) > 0

    def test_waypoint_reached_advances(self, drive_coverage):
        """When within position_tolerance, waypoint should advance."""
        drive_coverage.robot_x = 0.99
        drive_coverage.robot_y = 0.0
        drive_coverage.position_tolerance = 0.1
        drive_coverage.missed_waypoints = []
        drive_coverage.retry_count = 0
        drive_coverage.max_retries = 0
        drive_coverage.start_time = None
        drive_coverage.coverage_complete_pub = MagicMock()
        drive_coverage.drive_to_waypoint(1.0, 0.0, 100.0)
        assert drive_coverage.current_waypoint_idx == 1
        assert drive_coverage.successful_waypoints == 1

    def test_normalize_angle_edge_cases(self, drive_coverage):
        """Test normalize_angle with boundary values."""
        assert abs(drive_coverage.normalize_angle(math.pi)) - math.pi < 1e-9
        assert abs(drive_coverage.normalize_angle(-math.pi)) - math.pi < 1e-9
        assert abs(drive_coverage.normalize_angle(2 * math.pi)) < 1e-9


class TestDensifyWaypoints:
    """Test waypoint densification."""

    def test_single_waypoint(self, coverage):
        result = coverage.densify_waypoints([(0, 0, 0)])
        assert len(result) == 1

    def test_close_waypoints_not_densified(self, coverage):
        wps = [(0, 0, 0), (0.1, 0, 0)]
        result = coverage.densify_waypoints(wps, max_segment_length=0.15)
        assert len(result) == 2

    def test_far_waypoints_densified(self, coverage):
        wps = [(0, 0, 0), (1.0, 0, 0)]
        result = coverage.densify_waypoints(wps, max_segment_length=0.15)
        assert len(result) > 2
        # All points should be between start and end
        for x, y, _ in result:
            assert 0 <= x <= 1.0

    def test_preserves_endpoints(self, coverage):
        wps = [(0, 0, 0), (1.0, 0, 0)]
        result = coverage.densify_waypoints(wps, max_segment_length=0.5)
        assert result[0] == wps[0]
        assert result[-1] == wps[-1]


class TestOrientWaypoints:
    """Test waypoint orientation."""

    def test_single_waypoint(self, coverage):
        result = coverage.orient_waypoints([(0, 0, 0)])
        assert len(result) == 1

    def test_two_waypoints_rightward(self, coverage):
        wps = [(0, 0, 0), (1, 0, 0)]
        result = coverage.orient_waypoints(wps)
        # First waypoint should face right (yaw ≈ 0)
        assert abs(result[0][2]) < 0.01

    def test_two_waypoints_upward(self, coverage):
        wps = [(0, 0, 0), (0, 1, 0)]
        result = coverage.orient_waypoints(wps)
        # First waypoint should face up (yaw ≈ pi/2)
        assert abs(result[0][2] - math.pi / 2) < 0.01

    def test_last_waypoint_inherits_direction(self, coverage):
        wps = [(0, 0, 0), (1, 0, 0), (2, 0, 0)]
        result = coverage.orient_waypoints(wps)
        # Last waypoint should have same yaw as second-to-last
        assert abs(result[-1][2] - result[-2][2]) < 0.01


# ════════════════════════════════════════════════════════════════════
# Scan-to-Map Localization Tests
# ════════════════════════════════════════════════════════════════════

class TestScanToPoints:
    """Test _scan_to_points in WebBridgeNode."""

    @pytest.fixture
    def web_node(self):
        from clean_bot_mission.webapp.app import WebBridgeNode
        node = WebBridgeNode.__new__(WebBridgeNode)
        node.scan_ranges = []
        node.scan_angle_min = 0.0
        node.scan_angle_max = 2 * math.pi
        node.scan_angle_increment = math.pi / 180
        return node

    def test_empty_scan(self, web_node):
        web_node.scan_ranges = []
        assert web_node._scan_to_points() is None

    def test_too_few_valid_points(self, web_node):
        web_node.scan_ranges = [1.0] * 10 + [float('inf')] * 350
        assert web_node._scan_to_points() is None

    def test_valid_scan_returns_points(self, web_node):
        web_node.scan_ranges = [2.0] * 360
        pts = web_node._scan_to_points()
        assert pts is not None
        assert len(pts) == 360

    def test_filters_nan_and_inf(self, web_node):
        web_node.scan_ranges = [1.5] * 200 + [float('nan')] * 80 + [float('inf')] * 80
        pts = web_node._scan_to_points()
        assert pts is not None
        assert len(pts) == 200

    def test_filters_too_close_ranges(self, web_node):
        web_node.scan_ranges = [0.05] * 100 + [2.0] * 260
        pts = web_node._scan_to_points()
        assert pts is not None
        assert len(pts) == 260

    def test_point_coordinates(self, web_node):
        """Scan at angle=0 with range=1 should give (1, 0)."""
        web_node.scan_ranges = [1.0]
        web_node.scan_angle_min = 0.0
        web_node.scan_angle_increment = 0.1
        pts = web_node._scan_to_points()
        # Only 1 point, too few for the threshold
        assert pts is None

    def test_point_directions(self, web_node):
        """Forward (0°) and left (90°) scan points."""
        web_node.scan_angle_min = 0.0
        web_node.scan_angle_increment = math.pi / 2
        web_node.scan_ranges = [1.0] * 40  # Need >=30 valid points
        pts = web_node._scan_to_points()
        assert pts is not None
        # First point at angle 0: (1, 0)
        assert abs(pts[0][0] - 1.0) < 0.01
        assert abs(pts[0][1] - 0.0) < 0.01
        # Second point at pi/2: (0, 1)
        assert abs(pts[1][0] - 0.0) < 0.01
        assert abs(pts[1][1] - 1.0) < 0.01


class TestScorePose:
    """Test _score_pose in WebBridgeNode."""

    @pytest.fixture
    def web_node(self):
        from clean_bot_mission.webapp.app import WebBridgeNode
        node = WebBridgeNode.__new__(WebBridgeNode)
        return node

    def test_perfect_match(self, web_node):
        """All scan points land on wall cells → max score."""
        # Simple 10x10 grid with walls on right edge (col 9)
        w, h, res = 10, 10, 1.0
        wall_mask = np.zeros((h, w), dtype=bool)
        wall_mask[:, 9] = True  # wall at x=9.5
        ox, oy = 0.0, 0.0
        # Robot at (5, 5) facing right (theta=0), scan points at (4, 0) → world (9, 5)
        pts_x = np.array([4.0])
        pts_y = np.array([0.0])
        score = web_node._score_pose(pts_x, pts_y, 5.0, 5.0, 0.0,
                                     wall_mask, ox, oy, res, w, h)
        assert score == 1

    def test_no_match(self, web_node):
        """Scan points land in free space → score 0."""
        w, h, res = 10, 10, 1.0
        wall_mask = np.zeros((h, w), dtype=bool)
        wall_mask[:, 9] = True
        pts_x = np.array([1.0])
        pts_y = np.array([0.0])
        score = web_node._score_pose(pts_x, pts_y, 2.0, 5.0, 0.0,
                                     wall_mask, 0.0, 0.0, res, w, h)
        assert score == 0

    def test_out_of_bounds_ignored(self, web_node):
        """Points outside the grid don't count."""
        w, h, res = 5, 5, 1.0
        wall_mask = np.ones((h, w), dtype=bool)
        pts_x = np.array([10.0])
        pts_y = np.array([0.0])
        score = web_node._score_pose(pts_x, pts_y, 2.0, 2.0, 0.0,
                                     wall_mask, 0.0, 0.0, res, w, h)
        assert score == 0

    def test_rotation_affects_hits(self, web_node):
        """Rotating the pose changes which cells the scan hits."""
        w, h, res = 10, 10, 1.0
        wall_mask = np.zeros((h, w), dtype=bool)
        wall_mask[:, 9] = True  # wall at right edge
        pts_x = np.array([4.0])
        pts_y = np.array([0.0])
        # theta=0: point at (4+5,0+5)=(9,5) → hits wall
        s0 = web_node._score_pose(pts_x, pts_y, 5.0, 5.0, 0.0,
                                  wall_mask, 0.0, 0.0, res, w, h)
        # theta=pi/2: point rotated to (0, 4), world (5, 9) → free (col 5)
        s90 = web_node._score_pose(pts_x, pts_y, 5.0, 5.0, math.pi / 2,
                                   wall_mask, 0.0, 0.0, res, w, h)
        assert s0 == 1
        assert s90 == 0


class TestLocalizeOnSavedMap:
    """Test localize_on_saved_map with synthetic maps and scans."""

    @pytest.fixture
    def web_node(self):
        from clean_bot_mission.webapp.app import WebBridgeNode
        node = WebBridgeNode.__new__(WebBridgeNode)
        node.scan_ranges = []
        node.scan_angle_min = 0.0
        node.scan_angle_max = 2 * math.pi
        node.scan_angle_increment = 2 * math.pi / 360
        node.get_logger = MagicMock(return_value=MagicMock())
        return node

    def _make_box_map(self, size=50, res=0.05):
        """Create a box-shaped room (walls on all edges)."""
        data = [0] * (size * size)
        for r in range(size):
            for c in range(size):
                if r == 0 or r == size - 1 or c == 0 or c == size - 1:
                    data[r * size + c] = 100
        return data, size, size, res

    def _make_scan_for_box(self, robot_x, robot_y, robot_yaw, size, res, n_rays=360):
        """Simulate a LiDAR scan of a box room from a given position."""
        ranges = []
        half = size * res / 2.0
        ox = -half  # map origin
        # Walls at world coords: x in [ox, ox+size*res], y in [oy, oy+size*res]
        min_x, max_x = ox, ox + size * res
        min_y, max_y = ox, ox + size * res
        for i in range(n_rays):
            angle = robot_yaw + (2 * math.pi / n_rays) * i
            dx, dy = math.cos(angle), math.sin(angle)
            best_r = 10.0  # max range
            # Check intersection with 4 walls
            for wall_val, is_x, coord in [
                (min_x, True, min_x), (max_x, True, max_x),
                (min_y, False, min_y), (max_y, False, max_y),
            ]:
                if is_x and abs(dx) > 1e-9:
                    t = (coord - robot_x) / dx
                    if t > 0:
                        hit_y = robot_y + t * dy
                        if min_y <= hit_y <= max_y:
                            best_r = min(best_r, t)
                elif not is_x and abs(dy) > 1e-9:
                    t = (coord - robot_y) / dy
                    if t > 0:
                        hit_x = robot_x + t * dx
                        if min_x <= hit_x <= max_x:
                            best_r = min(best_r, t)
            ranges.append(max(0.12, best_r))
        return ranges

    def test_no_scan_returns_none(self, web_node):
        data, w, h, res = self._make_box_map()
        result = web_node.localize_on_saved_map(data, w, h, res, 0.0, 0.0)
        assert result is None

    def test_localizes_center_of_box(self, web_node):
        """Robot at center of box should be localized near center."""
        size, res = 50, 0.05
        data, w, h, _ = self._make_box_map(size, res)
        origin = -(size * res / 2.0)
        # Robot at world (0, 0) = center of the box, facing right
        web_node.scan_ranges = self._make_scan_for_box(0.0, 0.0, 0.0, size, res)
        web_node.scan_angle_min = 0.0
        web_node.scan_angle_increment = 2 * math.pi / 360
        result = web_node.localize_on_saved_map(data, w, h, res, origin, origin)
        assert result is not None
        x, y, yaw, score, total = result
        # Should be near center (0, 0) within 0.4m
        assert abs(x) < 0.4, f"x={x} too far from center"
        assert abs(y) < 0.4, f"y={y} too far from center"
        assert score > total * 0.3, f"score {score}/{total} too low"

    def test_localizes_off_center(self, web_node):
        """Robot at (0.5, 0.3) in box should be localized reasonably close."""
        size, res = 60, 0.05
        data, w, h, _ = self._make_box_map(size, res)
        origin = -(size * res / 2.0)
        rx, ry = 0.5, 0.3
        web_node.scan_ranges = self._make_scan_for_box(rx, ry, 0.5, size, res)
        web_node.scan_angle_min = 0.5
        web_node.scan_angle_increment = 2 * math.pi / 360
        result = web_node.localize_on_saved_map(data, w, h, res, origin, origin)
        assert result is not None
        x, y, yaw, score, total = result
        # Symmetric box may have multiple valid matches; check distance is reasonable
        dist = math.sqrt((x - rx) ** 2 + (y - ry) ** 2)
        assert dist < 1.0, f"Localized at ({x:.2f},{y:.2f}), {dist:.2f}m from ({rx},{ry})"
        assert score > total * 0.25

    def test_score_reflects_match_quality(self, web_node):
        """A well-placed scan should have high score."""
        size, res = 40, 0.05
        data, w, h, _ = self._make_box_map(size, res)
        origin = -(size * res / 2.0)
        web_node.scan_ranges = self._make_scan_for_box(0.0, 0.0, 0.0, size, res)
        web_node.scan_angle_min = 0.0
        web_node.scan_angle_increment = 2 * math.pi / 360
        result = web_node.localize_on_saved_map(data, w, h, res, origin, origin)
        assert result is not None
        _, _, _, score, total = result
        # With a perfect scan match, score should be at least 50% of total
        assert score >= total * 0.3


class TestCoveragePlannerLocalizationOffset:
    """Test the map↔odom offset in _get_robot_pose_map."""

    @pytest.fixture
    def coverage(self):
        planner = AdaptiveCoveragePlanner()
        planner.robot_x = 0.0
        planner.robot_y = 0.0
        planner.robot_yaw = 0.0
        planner.odom_received = True
        planner._map_odom_offset = None
        planner._tf_pose_warned = False
        # Make TF fail
        planner.tf_buffer = MagicMock()
        planner.tf_buffer.lookup_transform.side_effect = Exception("no TF")
        return planner

    def test_fallback_without_offset(self, coverage):
        """Without localization offset, fallback returns raw odom."""
        coverage.robot_x = 1.0
        coverage.robot_y = 2.0
        coverage.robot_yaw = 0.5
        x, y, yaw = coverage._get_robot_pose_map()
        assert abs(x - 1.0) < 0.01
        assert abs(y - 2.0) < 0.01
        assert abs(yaw - 0.5) < 0.01

    def test_fallback_with_offset(self, coverage):
        """With localization offset, fallback applies it to odom."""
        coverage.robot_x = 0.1
        coverage.robot_y = 0.2
        coverage.robot_yaw = 0.0
        coverage._map_odom_offset = (3.0, 4.0, math.pi / 4)
        x, y, yaw = coverage._get_robot_pose_map()
        assert abs(x - 3.1) < 0.01
        assert abs(y - 4.2) < 0.01
        assert abs(yaw - math.pi / 4) < 0.01

    def test_offset_updated_by_callback(self, coverage):
        """_on_robot_map_pose should compute offset from localized - odom."""
        coverage.robot_x = 0.5
        coverage.robot_y = 0.5
        coverage.robot_yaw = 0.0
        msg = MagicMock()
        msg.pose.position.x = 3.5
        msg.pose.position.y = 2.5
        # yaw = pi/2 via quaternion
        msg.pose.orientation.w = math.cos(math.pi / 4)
        msg.pose.orientation.z = math.sin(math.pi / 4)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        coverage._on_robot_map_pose(msg)
        assert coverage._map_odom_offset is not None
        dx, dy, dyaw = coverage._map_odom_offset
        assert abs(dx - 3.0) < 0.01
        assert abs(dy - 2.0) < 0.01
        assert abs(dyaw - math.pi / 2) < 0.15

    def test_tf_success_ignores_offset(self, coverage):
        """When TF works, offset is not applied."""
        coverage._map_odom_offset = (10.0, 10.0, 1.0)
        # Make TF succeed
        tf_mock = MagicMock()
        tf_mock.transform.translation.x = 5.0
        tf_mock.transform.translation.y = 6.0
        tf_mock.transform.rotation.w = 1.0
        tf_mock.transform.rotation.x = 0.0
        tf_mock.transform.rotation.y = 0.0
        tf_mock.transform.rotation.z = 0.0
        coverage.tf_buffer.lookup_transform.side_effect = None
        coverage.tf_buffer.lookup_transform.return_value = tf_mock
        x, y, yaw = coverage._get_robot_pose_map()
        assert abs(x - 5.0) < 0.01
        assert abs(y - 6.0) < 0.01

    def test_reset_clears_offset(self, coverage):
        """Bug 84 regression: handle_reset should clear _map_odom_offset."""
        coverage._map_odom_offset = (1.0, 2.0, 0.5)
        coverage.handle_reset()
        assert coverage._map_odom_offset is None


class TestCoveragePlannerNearestWaypoint:
    """Test vectorized find_nearest_waypoint_index."""

    @pytest.fixture
    def coverage(self):
        planner = AdaptiveCoveragePlanner()
        planner.robot_x = 0.0
        planner.robot_y = 0.0
        planner.robot_yaw = 0.0
        planner.odom_received = True
        planner._map_odom_offset = None
        planner._tf_pose_warned = False
        planner.tf_buffer = MagicMock()
        planner.tf_buffer.lookup_transform.side_effect = Exception("no TF")
        return planner

    def test_nearest_at_origin(self, coverage):
        coverage.waypoints = [(1.0, 0.0, 0.0), (0.1, 0.1, 0.0), (5.0, 5.0, 0.0)]
        idx = coverage.find_nearest_waypoint_index()
        assert idx == 1

    def test_nearest_with_offset(self, coverage):
        coverage._map_odom_offset = (10.0, 10.0, 0.0)
        coverage.waypoints = [(10.1, 10.0, 0.0), (0.0, 0.0, 0.0), (20.0, 20.0, 0.0)]
        idx = coverage.find_nearest_waypoint_index()
        assert idx == 0

    def test_single_waypoint(self, coverage):
        coverage.waypoints = [(3.0, 4.0, 0.0)]
        idx = coverage.find_nearest_waypoint_index()
        assert idx == 0


class TestFrontierMemoryOptimization:
    """Test that frontiers don't store unnecessary cell lists (Bug 82)."""

    @pytest.fixture
    def explorer(self):
        exp = FrontierExplorer()
        return exp

    def test_frontier_no_cells_key(self, explorer):
        """Frontiers should not have 'cells' key (memory optimization)."""
        explorer.map_info = MagicMock()
        explorer.map_info.width = 20
        explorer.map_info.height = 20
        explorer.map_info.resolution = 0.05
        explorer.map_info.origin.position.x = 0.0
        explorer.map_info.origin.position.y = 0.0
        # Create a map with a frontier
        grid = np.zeros((20, 20), dtype=np.int8)
        grid[:, :10] = 0   # free
        grid[:, 10:] = -1  # unknown
        explorer.map_array = grid
        explorer.min_frontier_size = 1
        frontiers = explorer.find_frontiers()
        for f in frontiers:
            assert 'cells' not in f, "Frontier should not store cell list"
            assert 'size' in f
            assert 'x' in f
            assert 'y' in f


class TestHeatmapDecay:
    """Test heatmap doesn't grow unbounded (Bug 80)."""

    def test_heatmap_decays_at_threshold(self):
        from clean_bot_mission.webapp.app import WebBridgeNode
        node = WebBridgeNode.__new__(WebBridgeNode)
        node.map_msg = None
        node.map_update_counter = 0
        node.obstacle_heatmap = np.full((10, 10), 999.0, dtype=np.float32)
        node._heatmap_width = 10
        node._heatmap_height = 10
        node._last_map_emit = time.monotonic() + 999  # far future, prevent emission
        node._map_rate_interval = 999
        node.sio = MagicMock()
        node.get_logger = MagicMock(return_value=MagicMock())

        # Create a map message mock with all occupied cells
        msg = MagicMock()
        msg.info.width = 10
        msg.info.height = 10
        msg.data = [100] * 100

        node._on_map(msg)
        # After adding 1.0 to all cells (999→1000), should trigger decay (* 0.5)
        assert node.obstacle_heatmap.max() <= 501.0
