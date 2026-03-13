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

### Bug 30: Missing rclpy.ok() guard in rplidar_test and test_helper
- **Files:** `clean_bot_hardware/clean_bot_hardware/rplidar_test.py`, `clean_bot_mission/scripts/test_helper.py`
- **Problem:** `rclpy.shutdown()` called unconditionally in `finally`/timer callbacks. If rclpy was already shut down (e.g., by KeyboardInterrupt handler or another node), this would raise `RuntimeError`. Same pattern as Bug 24.
- **Fix:** Added `if rclpy.ok():` guard before all `rclpy.shutdown()` calls.

### Bug 31: Division by zero in rplidar_test diagnostics
- **File:** `clean_bot_hardware/clean_bot_hardware/rplidar_test.py` (`display_diagnostics`)
- **Problem:** `100*valid_count/len(msg.ranges)` would crash with `ZeroDivisionError` if a LiDAR scan message arrived with an empty `ranges` array (e.g., during sensor startup or disconnection).
- **Fix:** Changed to `100*valid_count/max(len(msg.ranges),1)` to avoid division by zero.

### Bug 32: Bare except clauses in check_lidar diagnostic script
- **File:** `clean_bot_hardware/scripts/check_lidar.py`
- **Problem:** Two bare `except:` clauses that would catch `KeyboardInterrupt` and `SystemExit`, preventing clean script termination. Same pattern as Bug 23.
- **Fix:** Changed to `except Exception:` in both locations.

### Bug 33: LiDAR frame_id inconsistency — `laser_frame` vs `laser`
- **Files:** `clean_bot_hardware/config/rplidar_a1.yaml`, `clean_bot_hardware/launch/rplidar_test.launch.py`, `clean_bot_hardware/scripts/start_lidar.sh`, `clean_bot_description/urdf/lidar.xacro`
- **Problem:** The URDF defines the LiDAR link as `laser` (line 28 of lidar.xacro), and the main launch file (`robot_bringup.launch.py`) + Nav2 costmap config both use `laser`. However, `rplidar_a1.yaml`, `rplidar_test.launch.py`, and `start_lidar.sh` all used `laser_frame`. If someone launched the LiDAR using these files directly (not through the main bringup), the TF frames wouldn't connect — SLAM and Nav2 would fail because the scan message frame_id wouldn't match the URDF link name.
- **Fix:** Changed all occurrences of `laser_frame` to `laser` across config, launch, script, and URDF files. Also updated `docs/Clean_Bot_Flow_Diagrams.md` TF tree diagram. Added a test (`test_lidar_frame_id_consistent`) to catch future regressions.

### Bug 34: Arduino driver cmd_vel watchdog never implemented (safety)
- **File:** `clean_bot_hardware/clean_bot_hardware/arduino_driver.py`
- **Problem:** The driver stored `last_cmd_time` for a "watchdog" but never actually checked it. If the navigation stack or web app died while the robot was moving, the robot would keep driving at the last velocity indefinitely — a physical safety hazard.
- **Fix:** Added a watchdog check in `update_loop()` that sends `0,0` (stop motors) to the Arduino if no `cmd_vel` message arrives within 0.5 seconds. Tracks `_motors_stopped` to avoid spamming stop commands.

### Bug 35: Room names with single quotes break UI onclick handlers
- **File:** `clean_bot_mission/clean_bot_mission/webapp/templates/index.html` (`esc()` helper)
- **Problem:** The `esc()` HTML escaper used `textContent → innerHTML` which escapes `<>&"` but NOT single quotes (`'`). Room names with single quotes (e.g., "Bob's Room") would break inline `onclick="renameRoom('...')"` attributes, causing JavaScript syntax errors and making those room buttons non-functional.
- **Fix:** Added `.replace(/'/g, '&#39;')` to the `esc()` function to also escape single quotes for safe use in HTML attribute contexts.

### Bug 36: Room preview crashes on corrupted data length
- **File:** `clean_bot_mission/clean_bot_mission/webapp/app.py` (`load_room_preview`)
- **Problem:** `load_room_preview` checked for missing fields but not data array length. A corrupted room file where `len(data) != width * height` would crash with `numpy.reshape` ValueError instead of returning `None` gracefully. Also, `resolution` field was not in the required-fields check.
- **Fix:** Added validation: `w <= 0 or h <= 0 or len(data) != w * h` returns `None`. Added `resolution` to required fields list.

### Bug 37: Room save/rename with all-special-characters name creates '.json' file
- **Files:** `clean_bot_mission/clean_bot_mission/webapp/app.py` (`save_room`, `rename_room`)
- **Problem:** If a room name consisted entirely of special characters (e.g., `@#$%`), the filename sanitization would produce an empty string, resulting in a file named `.json` — a hidden file that would be hard to manage. Same issue in `rename_room`.
- **Fix:** Added `if not safe_name:` guard after sanitization, returning a clear error message asking for at least one alphanumeric character.

### Bug 38: Path trail stops updating after list truncation
- **File:** `clean_bot_mission/clean_bot_mission/webapp/app.py` (`_update_pose`)
- **Problem:** When `path_trail` exceeds 500 items and gets truncated to the last 500, `_trail_sent_index` was not adjusted. It stayed at ~500 while the new list also has 500 items, so `path_trail[500:]` was always empty — no new trail points were ever sent to the frontend after the first truncation.
- **Fix:** When truncating, adjust `_trail_sent_index` by subtracting the number of removed items: `self._trail_sent_index = max(0, self._trail_sent_index - removed)`.

### Bug 39: Double map_update_counter increment in load_and_clean_room
- **File:** `clean_bot_mission/clean_bot_mission/webapp/app.py` (`load_and_clean_room`)
- **Problem:** `load_and_clean_room` manually set `self.map_msg = msg` and incremented `self.map_update_counter`, then published the message to `/map`. Since the node also subscribes to `/map`, the `_on_map` callback would fire and do the same updates again — double-incrementing the counter and double-processing the heatmap.
- **Fix:** Removed the manual `self.map_msg` assignment and `map_update_counter` increment from `load_and_clean_room`. Let the `_on_map` callback handle internal state updates when it receives the published message.

### Bug 40: Many dynamic UI strings not translated
- **File:** `clean_bot_mission/clean_bot_mission/webapp/templates/index.html`
- **Problem:** Toast messages, the rename prompt, modal metadata text, emergency stop message, navigation messages, and export map messages were all hardcoded in English. Switching to Hebrew left ~16 strings untranslated.
- **Fix:** Added 16 new i18n keys to both `en` and `he` dictionaries (`room_saved`, `room_deleted`, `room_renamed`, `rename_prompt`, `rename_failed`, `save_failed`, `preview_failed`, `enter_room_name`, `command_sent`, `confirm_action`, `emergency_activated`, `navigating_to`, `nav_failed`, `map_exported`, `no_map_export`, `failed`). Updated all dynamic strings to use `t()` function.

### Bug 41: AudioContext leak in notification chime
- **File:** `clean_bot_mission/clean_bot_mission/webapp/templates/index.html` (`playChime`)
- **Problem:** A new `AudioContext` was created every time `playChime()` was called. AudioContexts are limited browser resources (browsers cap at ~6-8 per page) and creating too many causes silent failure or console warnings.
- **Fix:** Changed to singleton pattern: store `_audioCtx` in module scope, create on first use, reuse for subsequent calls.

### Bug 42: Banner alignment in full_mission.py uses hardcoded spaces
- **File:** `clean_bot_mission/clean_bot_mission/full_mission.py` (`print_banner`, `finish_mission`)
- **Problem:** Banner text was padded with hardcoded spaces, causing misaligned borders when text content had different widths. The right-edge `║` characters didn't line up.
- **Fix:** Added `pad(text)` helper that uses `ljust()` to dynamically pad each line to the banner width, ensuring consistent alignment.

### Bug 43: CRITICAL — arduino_driver.py missing `def _wheel_speed_to_pwm(` line
- **File:** `clean_bot_hardware/clean_bot_hardware/arduino_driver.py`
- **Problem:** The `def _wheel_speed_to_pwm(` method definition line was completely missing. The method's parameters (`self, wheel_speed_mps: float, ...`) were orphaned inside the body of `cmd_vel_callback`, causing a **syntax error** (`unmatched ')'`). The file could not be imported at all, meaning the Arduino motor driver node could not start on the robot.
- **Fix:** Restored the missing `def _wheel_speed_to_pwm(` line and proper blank-line separator between `cmd_vel_callback` and the new method.

### Bug 44: Emergency stop provides no protection when sensor never publishes
- **File:** `clean_bot_hardware/clean_bot_hardware/emergency_stop.py`
- **Problem:** On startup, `current_distance` defaults to 100m and `last_range_time` is `None`. If the ultrasonic sensor is disconnected, broken, or has a wrong topic name, the timeout check is skipped entirely (it only triggers when `last_range_time is not None`). The robot operates with **zero collision protection** — driving at full speed into walls.
- **Fix:** Added `_start_time` and `_sensor_grace_sec` (5s). After the grace period, if no sensor data has ever been received, forward speed is halved and a warning is logged. This matches the existing sensor-timeout behavior.

### Bug 45: RViz marker memory leak in low_obstacle_detector
- **File:** `clean_bot_hardware/clean_bot_hardware/low_obstacle_detector.py`
- **Problem:** Obstacle visualization markers were published with `lifetime.sec = 0` and `lifetime.nanosec = 0`, meaning "never expire" in RViz. Combined with an ever-incrementing `marker_id_counter`, markers accumulated indefinitely in RViz, consuming increasing memory.
- **Fix:** Changed marker lifetime to match the `obstacle_persistence` parameter (default 2.0s). Markers now automatically expire when the obstacle detection window closes.

### Bug 46: WebSocket handlers crash on non-dict data
- **File:** `clean_bot_mission/clean_bot_mission/webapp/app.py` (ws_command, ws_velocity, ws_latency_ping, ws_set_map_rate)
- **Problem:** All WebSocket event handlers called `data.get()` without verifying that `data` is a dict. If a client sends a string, int, or None, the handlers raise `AttributeError`. While Flask-SocketIO catches the exception (server doesn't crash), it fills logs with tracebacks.
- **Fix:** Added `if not isinstance(data, dict): return` guard to all affected handlers. Also added try/except for `float()` conversion in `ws_set_map_rate`.

### Bug 47: Mission log only updates on page refresh
- **File:** `clean_bot_mission/clean_bot_mission/webapp/app.py` (`send_command`)
- **Problem:** `send_command()` appended log entries to `self.mission_log` (deque) but never emitted them via WebSocket. The frontend only received log entries via the `mission_state` event, which fires on ROS state transitions — not on commands. Commands like "Command sent: start_scan" were invisible until page refresh triggered `GET /api/log`.
- **Fix:** Added `self.sio.emit("mission_state", {"state": self.mission_state, "log_entry": entry})` after appending the log entry, so the frontend receives real-time updates.

### Bug 48: Light mode shows dark-themed boxes
- **File:** `clean_bot_mission/clean_bot_mission/webapp/templates/index.html` (CSS)
- **Problem:** Many elements used hardcoded dark hex colors (`#1e1f36`, `#353660`, `#1a1b30`, `#444`, `#333`, `#16172a`, `#2a2b44`, `#282830`) that the light theme toggle didn't override. Inputs, buttons, log boxes, map containers, progress bars, and scrollbar thumbs all stayed dark in light mode.
- **Fix:** Introduced 12 new CSS custom properties (`--input-bg`, `--input-border`, `--btn-bg`, `--btn-hover`, `--log-bg`, `--log-border`, `--map-bg`, `--map-border`, `--bar-bg`, `--legend-border`, `--swatch-unknown`, `--header-bg`) in `:root` and `html.light`. Replaced all hardcoded colors with `var(...)` references. Removed redundant `html.light` overrides for buttons that now inherit from variables.

### Bug 49: Web app unreachable from other devices on LAN
- **File:** `clean_bot_mission/clean_bot_mission/webapp/templates/index.html` (script tag)
- **Problem:** The socket.io client library was loaded from `cdnjs.cloudflare.com`. On LAN-only networks without internet access, this CDN request fails and the entire app breaks — socket connections can't be established.
- **Fix:** Changed the script `src` to `/socket.io/socket.io.js`, which Flask-SocketIO 5.x serves automatically. No external CDN dependency needed.
