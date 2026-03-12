# Bug Fixes

All bug fixes applied during the project cleanup (`cleanup/streamline-project` branch).

---

## Critical Bugs

### 1. `return_home_sequence()` was a no-op placeholder
- **File:** `clean_bot_mission/clean_bot_mission/full_mission.py`
- **Problem:** The return-home function was just `time.sleep(1.0)` — the robot never actually navigated home.
- **Fix:** Implemented full Nav2 `NavigateToPose` action with async goal/result callbacks. Added `nav_client` ActionClient to `__init__`.

### 2. Emergency stop completely disabled
- **File:** `clean_bot_hardware/clean_bot_hardware/emergency_stop.py`
- **Problem:** The entire `cmd_vel_callback` had an early `return` after `self.cmd_vel_pub.publish(msg)`, bypassing all safety logic. Comment said "TEMPORARILY DISABLED" but it was committed.
- **Fix:** Removed the early return and added a configurable `enabled` parameter (default: `True`) so safety can be toggled via ROS parameter instead of code edits.

### 3. Telegram token hardcoded in source
- **File:** `clean_bot_mission/scripts/telegram_bridge.py`
- **Problem:** The Telegram bot token was hardcoded as a string literal in the script — a security risk.
- **Fix:** Changed to read from `CLEANBOT_TELEGRAM_TOKEN` environment variable. Script exits with helpful error message if the variable is not set.

---

## Initialization Bugs

### 4. Uninitialized attributes in `frontier_explorer.py`
- **File:** `clean_bot_mission/clean_bot_mission/frontier_explorer.py`
- **Problem:** `_retry_after_clear`, `_last_no_map_warn`, and `_no_map_warn_period_s` were used in methods but never initialized in `__init__`, causing `AttributeError` at runtime.
- **Fix:** Added initialization in `__init__`: `_retry_after_clear = False`, `_last_no_map_warn = 0.0`, `_no_map_warn_period_s = 5.0`.

### 5. Auto-start timer fired repeatedly instead of once
- **File:** `clean_bot_mission/clean_bot_mission/full_mission.py`
- **Problem:** `create_timer(3.0, self._auto_start_once)` creates a repeating timer. Despite the method name suggesting "once", it fired every 3 seconds, potentially re-triggering exploration.
- **Fix:** Store timer reference in `self._auto_start_timer` and cancel it inside `_auto_start_once()` after the first invocation.

---

## Logic Bugs

### 6. Cleaning hardware not deactivated on `go_home` / `reset`
- **File:** `clean_bot_mission/clean_bot_mission/full_mission.py`
- **Problem:** If the user sent `go_home` or `reset` while the robot was in COVERAGE state, the cleaning relay and servo remained active (vacuum running, brush spinning) as the robot drove home.
- **Fix:** Added `if self.state == MissionState.COVERAGE: self._deactivate_cleaning_hardware()` to both `handle_go_home()` and `handle_reset()`.

### 7. Dead code: `_velocity_to_pwm` in `arduino_driver.py`
- **File:** `clean_bot_hardware/clean_bot_hardware/arduino_driver.py`
- **Problem:** `_velocity_to_pwm()` method was unused — replaced by `_wheel_speed_to_pwm()` but never removed.
- **Fix:** Deleted the dead method (18 lines).

---

## Unused / Stale References

### 8. Unused `ekf_config` variable in launch file
- **File:** `clean_bot_hardware/launch/robot_bringup.launch.py`
- **Problem:** `ekf_config = os.path.join(...)` pointed to `ekf.yaml` which was never passed to any node (EKF is not used — rf2o provides odometry).
- **Fix:** Removed the variable and deleted the unused `ekf.yaml` config file.

### 9. Unused `ImageFont` import
- **File:** `clean_bot_mission/scripts/telegram_bridge.py`
- **Problem:** `from PIL import Image, ImageDraw, ImageFont` — `ImageFont` was imported but never used.
- **Fix:** Changed to `from PIL import Image, ImageDraw`.

### 10. Unused imports in `full_mission.py`
- **File:** `clean_bot_mission/clean_bot_mission/full_mission.py`
- **Problem:** `math`, `threading`, `PoseStamped`, and `OccupancyGrid` were imported but never used.
- **Fix:** Removed unused imports.
