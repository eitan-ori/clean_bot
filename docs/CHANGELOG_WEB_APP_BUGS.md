# Bug Fixes Changelog

## Bugs Found and Fixed

### Bug 1: Incomplete quaternion-to-yaw conversion in web app
- **File:** `clean_bot_mission/clean_bot_mission/webapp/app.py` (`_update_pose`)
- **Problem:** The yaw extraction from quaternion used only `qz` and `qw` components: `atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)`. This is missing the `qx` and `qy` cross-terms needed for the full conversion, producing incorrect yaw when roll or pitch are non-zero.
- **Fix:** Changed to the full formula: `atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))`.

### Bug 2: NoneType crash when API endpoints receive non-JSON requests
- **File:** `clean_bot_mission/clean_bot_mission/webapp/app.py` (multiple routes)
- **Problem:** `request.json.get(...)` is called in `api_command`, `api_velocity`, `api_save_room`, `api_rename_room`, and `api_navigate` without checking if `request.json` is `None`. When a request arrives without a JSON content-type header, `request.json` returns `None` and `.get()` raises `AttributeError`.
- **Fix:** Added `body = request.json or {}` before accessing attributes in all five affected endpoints.

### Bug 3: Path traversal vulnerability in room file operations
- **File:** `clean_bot_mission/clean_bot_mission/webapp/app.py` (`delete_room`, `load_room_preview`, `rename_room`)
- **Problem:** The `filename` parameter from user input was passed directly into `SAVED_ROOMS_DIR / f"{filename}.json"` without sanitization. An attacker could use `../../etc/passwd` or similar to read/delete files outside the saved_rooms directory.
- **Fix:** Added `_safe_room_path()` static method that strips non-alphanumeric characters (except `-` and `_`), rejects mismatched filenames, and verifies the resolved path stays within `SAVED_ROOMS_DIR`. Applied to `delete_room`, `load_room_preview`, and `rename_room`.

### Bug 4: Division by zero in emergency stop slow-down zone
- **File:** `clean_bot_hardware/clean_bot_hardware/emergency_stop.py` (`cmd_vel_callback`)
- **Problem:** The speed scaling formula `(distance - self.stop_dist) / (self.slow_dist - self.stop_dist)` divides by zero when `slow_down_distance` equals `emergency_stop_distance` (both configurable parameters).
- **Fix:** Added a guard: compute `dist_range = self.slow_dist - self.stop_dist`; if `dist_range <= 0`, use `self.slow_factor` directly instead of dividing.

### Bug 5: Division by zero in simple coverage scan callback
- **File:** `clean_bot_mission/clean_bot_mission/simple_coverage.py` (`scan_callback`)
- **Problem:** `angle_per_reading = (msg.angle_max - msg.angle_min) / num_readings` divides by zero if `angle_max == angle_min` (degenerate scan). Then `front_readings = int(math.radians(60) / angle_per_reading / 2)` would also divide by zero.
- **Fix:** Added early return guard: `if angle_range <= 0 or num_readings < 2: return`.

### Bug 6: Missing rclpy.ok() check before shutdown in simple_coverage
- **File:** `clean_bot_mission/clean_bot_mission/simple_coverage.py` (`main`)
- **Problem:** `rclpy.shutdown()` was called unconditionally in the `finally` block. If rclpy was already shut down (e.g., by a signal handler), this raises an exception.
- **Fix:** Added `if rclpy.ok():` guard before `rclpy.shutdown()`, matching the pattern used in all other nodes.

### Bug 7: Bare except clause in Arduino driver cleanup
- **File:** `clean_bot_hardware/clean_bot_hardware/arduino_driver.py` (`destroy_node`)
- **Problem:** `except:` catches all exceptions including `SystemExit` and `KeyboardInterrupt`, which should propagate. This silently swallows critical Python exceptions.
- **Fix:** Changed to `except Exception:` to only catch standard exceptions.

### Bug 8: Serial port disconnection crash in Arduino update loop
- **File:** `clean_bot_hardware/clean_bot_hardware/arduino_driver.py` (`update_loop`)
- **Problem:** `self.serial.in_waiting` was accessed without checking if the serial port is still open. If the USB cable is unplugged at runtime, this raises `serial.SerialException` or `OSError`, crashing the node.
- **Fix:** Wrapped the entire read block in a `try/except` that checks `self.serial.is_open` first and catches both `serial.SerialException` and `OSError`.

### Bug 9: Uninitialized self.sensor attribute in IMU publisher
- **File:** `clean_bot_hardware/clean_bot_hardware/imu_publisher_node.py` (`__init__`, `publish_data`)
- **Problem:** If `SimpleIMU()` constructor raises an exception, `self.sensor` is never assigned. Although the `return` prevents timer creation in the normal flow, any future code changes or subclassing could call `publish_data()` and get `AttributeError: 'ImuPublisherNode' object has no attribute 'sensor'`.
- **Fix:** Initialize `self.sensor = None` before the try block, and add `if self.sensor is None: return` guard at the top of `publish_data()`.

### Bug 10: Blocking sleep and spin_once in adaptive coverage callback
- **File:** `clean_bot_mission/clean_bot_mission/adaptive_coverage.py` (`start_coverage_mission`)
- **Problem:** `time.sleep()` and `rclpy.spin_once(self)` were called inside `start_coverage_mission()`, which runs on the executor thread. `time.sleep()` blocks the executor, preventing all other callbacks from firing. `rclpy.spin_once(self)` on a node that's already being spun by a `MultiThreadedExecutor` (as in `full_mission.py`) raises "Node already added to an executor" errors.
- **Fix:** Removed the blocking wait loop entirely. If map is not yet available, the method now returns immediately with an error message. The map arrives asynchronously via `map_callback` and the user can retry the start command.

### Bug 11: Missing OSError handling in room save
- **File:** `clean_bot_mission/clean_bot_mission/webapp/app.py` (`save_room`)
- **Problem:** The `except` clause only caught `TypeError`, `ValueError`, and `OverflowError` for JSON serialization errors. Filesystem errors like full disk, permission denied, or read-only filesystem (`OSError`) were not caught and would crash the endpoint with a 500 error and no useful message.
- **Fix:** Added `OSError` to the exception tuple.

### Bug 12: Missing error handling in room preview loading
- **File:** `clean_bot_mission/clean_bot_mission/webapp/app.py` (`load_room_preview`)
- **Problem:** `json.load(f)` was called with no exception handling. If a saved room file is corrupted or contains invalid JSON, this raises `json.JSONDecodeError` and crashes the endpoint.
- **Fix:** Wrapped the file open and JSON load in `try/except (json.JSONDecodeError, OSError)` with a warning log and `None` return.
