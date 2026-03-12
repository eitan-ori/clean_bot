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

### Bug 13: WebSocket command handler has no validation
- **File:** `clean_bot_mission/clean_bot_mission/webapp/app.py` (`ws_command`)
- **Problem:** The WebSocket `command` event handler forwarded any string directly to `send_command()` without validation. While the HTTP route validated against a whitelist of 8 commands, the WebSocket route did not. Malicious or buggy clients could send arbitrary strings to `/mission_command`.
- **Fix:** Added validation against `VALID_COMMANDS` set (shared with the HTTP route) before calling `send_command()`.

### Bug 14: Race condition on mission_log list
- **File:** `clean_bot_mission/clean_bot_mission/webapp/app.py` (`_on_mission_state`, `api_log`)
- **Problem:** `mission_log` was a plain list appended from the ROS callback thread and read from Flask threads. The truncation `self.mission_log = self.mission_log[-200:]` created a window where another thread's append could be lost. List reassignment is not atomic with the preceding read.
- **Fix:** Changed `mission_log` from `list` to `collections.deque(maxlen=200)` which is thread-safe for append/iterate and auto-truncates.

### Bug 15: Race condition on path_trail list
- **File:** `clean_bot_mission/clean_bot_mission/webapp/app.py` (`_update_pose`, `_push_status`, `api_trail`)
- **Problem:** `path_trail` was modified from the ROS timer thread (`_update_pose`) and read from Flask threads (`api_trail`, `_push_status`). The reassignment `self.path_trail = trail[-500:]` could lose concurrent appends.
- **Fix:** Added `_trail_lock` (threading.Lock) protecting all reads and writes to `path_trail` and `_trail_sent_index`.

### Bug 16: Room preview required ROS connection unnecessarily
- **File:** `clean_bot_mission/clean_bot_mission/webapp/app.py` (`api_room_preview`, `load_room_preview`)
- **Problem:** The room preview endpoint returned 503 when `ros_node is None`, but the operation only reads saved JSON files from disk — no ROS connection needed. Users couldn't view saved rooms when the robot was disconnected.
- **Fix:** Converted `load_room_preview` from instance method to `@staticmethod` and removed the `ros_node is None` check from the route.

### Bug 17: rename_room writes to old file before renaming
- **File:** `clean_bot_mission/clean_bot_mission/webapp/app.py` (`rename_room`)
- **Problem:** The method wrote updated JSON to the old file path, then renamed the file. If the rename failed (e.g., target exists, permissions), the old file would contain the new name in its JSON data, creating an inconsistency.
- **Fix:** Changed to write directly to the new path first, then delete the old file. If the paths are the same, write in place.

### Bug 18: rclpy.init() failure has no error message
- **File:** `clean_bot_mission/clean_bot_mission/webapp/app.py` (`main`)
- **Problem:** If ROS 2 was not installed or not sourced on the remote PC, `rclpy.init()` would throw an exception with a cryptic error. Users wouldn't know what went wrong.
- **Fix:** Wrapped in try/except with a clear error message explaining that ROS 2 must be installed and sourced.

### Bug 19: Test cross-contamination between integration and webapp tests
- **File:** `clean_bot_mission/test/test_integration.py`
- **Problem:** Integration tests assigned lambdas to `WebBridgeNode` class methods (`list_rooms`, `delete_room`, etc.) but didn't restore them, causing subsequent webapp tests to fail when run in the same pytest session.
- **Fix:** Used `_FakeNode` base class in integration tests so `WebBridgeNode` has real static methods. Added cleanup in the `client` fixture to restore any modified class attributes.

### Bug 20: Schedule checker can double-trigger within the same minute
- **File:** `clean_bot_mission/clean_bot_mission/webapp/app.py` (`_check_schedules`)
- **Problem:** The schedule checker runs every 60 seconds. If the timer fires twice within the same minute (e.g., at 08:30:05 and 08:30:50), the same schedule would match both times, sending `start_clean` twice. The robot might not have transitioned out of WAITING state yet between the two triggers.
- **Fix:** Added `_last_triggered` tracking per schedule. Each schedule stores a `day_time` key of its last trigger, and won't fire again if the key matches the current minute.

### Bug 21: Launch file port defaults mismatch
- **File:** `clean_bot_hardware/launch/robot_bringup.launch.py`
- **Problem:** The `LaunchConfiguration` defaults didn't match the `DeclareLaunchArgument` defaults. `arduino_port` defaulted to `/dev/ttyACM0` in LaunchConfiguration but `/dev/ttyUSB0` in DeclareLaunchArgument. `lidar_port` was `/dev/ttyUSB0` vs `/dev/ttyUSB1`. `wheel_radius` was `0.0335` vs `0.034`. Since `DeclareLaunchArgument` takes priority, the `LaunchConfiguration` defaults were misleading.
- **Fix:** Made all defaults consistent: `arduino_port=/dev/ttyACM0`, `lidar_port=/dev/ttyUSB0`, `wheel_radius=0.0335`.

### Bug 22: Telegram bridge mission-complete notification crashes
- **File:** `clean_bot_mission/scripts/telegram_bridge.py` (`_check_mission_complete`)
- **Problem:** `create_map_image()` returns a PIL `Image` object and an info string. The auto-notification handler passed the Image object to `io.BytesIO(img_bytes)`, treating it as raw bytes. This would crash with `TypeError` when a mission completed, preventing the user from receiving the final map.
- **Fix:** Changed to the correct pattern: create a BytesIO, call `image.save(bio, 'PNG')`, then `bio.seek(0)`.

### Bug 23: Bare except clause in IMU magnetometer read
- **File:** `clean_bot_hardware/clean_bot_hardware/simple_imu_driver.py` (`get_mag_data`)
- **Problem:** A bare `except:` clause caught ALL exceptions (including `KeyboardInterrupt`, `SystemExit`) silently, returning (0,0,0). This made it impossible to cleanly stop the driver and hid real errors like I2C bus failures.
- **Fix:** Changed to `except Exception as e:` with a descriptive error message.

### Bug 24: Missing rclpy.ok() check before shutdown in IMU broadcaster
- **File:** `clean_bot_hardware/clean_bot_hardware/imu_odom_broadcaster.py` (`main`)
- **Problem:** The `finally` block called `rclpy.shutdown()` unconditionally. If another node or a signal handler had already called `rclpy.shutdown()`, this would raise `RuntimeError: rclpy is already shutdown`.
- **Fix:** Added `if rclpy.ok():` guard before `rclpy.shutdown()`, consistent with all other nodes.

### Bug 25: Simple coverage bypasses emergency stop
- **File:** `clean_bot_mission/clean_bot_mission/simple_coverage.py`
- **Problem:** The simple coverage planner published velocity commands to `cmd_vel` directly, bypassing the emergency stop safety filter which subscribes to `cmd_vel_nav`. The adaptive coverage planner correctly uses `cmd_vel_nav`. This meant the robot could drive into obstacles during simple coverage.
- **Fix:** Changed publisher topic from `cmd_vel` to `cmd_vel_nav`.

### Bug 26: Coverage path skips waypoints before starting index
- **File:** `clean_bot_mission/clean_bot_mission/adaptive_coverage.py` (`start_coverage_mission`)
- **Problem:** Coverage starts from the nearest waypoint to the robot (`find_nearest_waypoint_index`), then increments the index sequentially until it reaches the end. All waypoints before the starting index (0..nearest-1) were never visited, meaning potentially large areas of the room were left uncleaned.
- **Fix:** Rotated the waypoint list so the nearest waypoint becomes index 0: `waypoints[nearest:] + waypoints[:nearest]`. The robot now starts from the nearest waypoint and covers all of them.

### Bug 27: TSP cell ordering starts from (0,0) instead of robot position
- **File:** `clean_bot_mission/clean_bot_mission/adaptive_coverage.py` (`generate_adaptive_coverage_path`)
- **Problem:** The nearest-neighbor TSP algorithm for ordering cells hardcoded `curr_pos = (0.0, 0.0)` as the starting position. After exploration, the robot is typically far from the origin, so the first cell visited could be across the room, wasting time on unnecessary travel.
- **Fix:** Changed to use the robot's actual position from TF/odometry: `rx, ry, _ = self._get_robot_pose_map()`.

### Bug 28: Nav2 costmap robot_radius inconsistent with actual robot
- **File:** `clean_bot_hardware/config/nav2_params.yaml`
- **Problem:** The local costmap used `robot_radius: 0.18` and the global costmap used `robot_radius: 0.12`, but the actual robot body radius is `0.20m`. The global costmap's 12cm radius was dangerously small — Nav2 would plan paths where the 20cm robot could collide with obstacles.
- **Fix:** Set both costmap `robot_radius` values to `0.20` to match the actual robot dimensions.

### Bug 29: Overlap ratio comment/code mismatch
- **File:** `clean_bot_mission/clean_bot_mission/adaptive_coverage.py`
- **Problem:** The `overlap_ratio` parameter was set to `0.1` (10%) but the inline comment said "15% overlap for safety". This could mislead developers into thinking the overlap is larger than it actually is.
- **Fix:** Corrected the comment to "10% overlap for safety".
