# Clean Bot — Full Mission Logic (Launch + Data Flow)

This document is the “source of truth” for how the Clean Bot autonomy stack fits together:

- Which launch files start which nodes
- Which nodes publish/subscribe to which topics
- Where TF frames come from (`map`, `odom`, `base_link`, `laser`)
- How **mission commands** (e.g., `start_scan`) turn into **robot motion** and **Arduino serial commands**

It is written for ROS 2 Humble-style systems.

---

## 1) Big Picture

There are three layers:

1. **Hardware + Localization + Navigation** (runs on the robot computer / Pi)
2. **Mission orchestration** (runs on the robot computer / Pi)
3. **Human interface** (PC via Telegram bridge, or CLI publishing to `/mission_command`)

High-level goal:

- User sends a string command (e.g. `start_scan`) → the mission controller enters `EXPLORING` → the explorer sends Nav2 goals → Nav2 outputs velocity (`/cmd_vel_nav`) → safety filter outputs `/cmd_vel_safe` → Arduino driver converts Twist to PWM and writes serial.

---

## 2) Launch Entry Points (What you actually run)

### 2.1 Scan-only (auto-start exploration)

- Launch: `ros2 launch clean_bot_mission scan_room.launch.py`
- Purpose: bring up hardware + SLAM + Nav2 and automatically start exploration.

Useful launch args for debugging:

- `start_explorer:=false` — disables `frontier_explorer` to reduce CPU/memory load while you debug SLAM stability.
- `explorer_delay_sec:=8.0` — delays starting `frontier_explorer` so Cartographer/Nav2 can initialize first.
- `auto_start_delay_sec:=10.0` — delays mission auto-start so the explorer doesn’t miss the one-shot `exploration_control=start` message.

What it starts:

- Includes hardware stack: `clean_bot_hardware/launch/robot_bringup.launch.py`
- Starts mission controller: `clean_bot_mission/full_mission` (with `auto_start:=true`)
- Starts explorer as a separate node: `clean_bot_mission/frontier_explorer`
- Starts a console monitor: `clean_bot_mission/scan_monitor`

Reference:
- `src/clean_bot_mission/launch/scan_room.launch.py`

### 2.2 Full mission (explore then clean)

- Launch: `ros2 launch clean_bot_mission cleaning_mission.launch.py`
- Purpose: bring up hardware + SLAM + Nav2 and wait for user commands.

What it starts:

- Includes hardware stack: `clean_bot_hardware/launch/robot_bringup.launch.py`
- Starts:
  - `clean_bot_mission/frontier_explorer` (unless `skip_exploration:=true`)
  - `clean_bot_mission/adaptive_coverage`
  - `clean_bot_mission/full_mission`

Reference:
- `src/clean_bot_mission/launch/cleaning_mission.launch.py`

### 2.3 Hardware-only bringup

- Launch: `ros2 launch clean_bot_hardware robot_bringup.launch.py`
- Purpose: start drivers + odometry + SLAM + Nav2 without mission.

Reference:
- `src/clean_bot_hardware/launch/robot_bringup.launch.py`

---

## 3) Hardware + Localization + Nav2 (clean_bot_hardware)

All of these are started by `robot_bringup.launch.py`.

### 3.1 Robot description + static TF

- Node: `robot_state_publisher`
- Publishes: TF from URDF (typically `base_link` → sensor frames like `laser`, `imu_link`, etc.)

Reference:
- `src/clean_bot_hardware/launch/robot_bringup.launch.py`

### 3.2 LiDAR: raw scan vs throttled scan

- Node: `sllidar_ros2/sllidar_node`
  - Publishes scans on `/scan` by default, but bringup remaps it to `/scan_raw`
- Node: `clean_bot_hardware/scan_throttle`
  - Subscribes: `/scan_raw`
  - Publishes: `/scan` at a reduced rate

Why there are two scan topics:

- `/scan_raw` is the “full-rate” stream used by odometry + SLAM.
- `/scan` is a “CPU-friendly” stream used by costmaps and other consumers.

References:
- `src/clean_bot_hardware/launch/robot_bringup.launch.py` (LiDAR + remap)
- `src/clean_bot_hardware/clean_bot_hardware/scan_throttle.py`

### 3.3 Laser odometry (rf2o)

- Node: `rf2o_laser_odometry/rf2o_laser_odometry_node`
- Subscribes: `/scan_raw`
- Publishes: `/odom` and TF `odom → base_link`

Important design note:

- This stack intentionally makes rf2o consume `/scan_raw` to avoid circular dependencies.

Reference:
- `src/clean_bot_hardware/launch/odom.launch.py`

### 3.4 SLAM (Cartographer)

- Node: `cartographer_ros/cartographer_node`
  - Subscribes: `/scan_raw`
  - Uses `/odom` (configured in `cartographer.lua`)
  - Publishes TF `map → odom` (Cartographer is configured to *not* provide `odom→base_link`)
- Node: `cartographer_ros/cartographer_occupancy_grid_node`
  - Publishes: `/map` (nav_msgs/OccupancyGrid)

Reference:
- `src/clean_bot_hardware/launch/cartographer.launch.py`
- `src/clean_bot_hardware/config/cartographer.lua`

Important packaging note:

- `cartographer.lua` uses Lua `include` to load shared defaults (`map_builder.lua`, `trajectory_builder.lua`, etc.).
- Those included files must exist in the same config directory that you pass via `-configuration_directory`.
- In this repo they live in: `src/clean_bot_hardware/config/`.

### 3.5 Nav2

Nav2 is started by including `nav2_bringup/launch/navigation_launch.py` with the params file:

- Params: `src/clean_bot_hardware/config/nav2_params.yaml`

Velocity topic conventions in this repo:

- Nav2’s “cmd_vel” is remapped so output ends up on: `/cmd_vel_nav`
- The safety layer listens to `/cmd_vel_nav` and outputs `/cmd_vel_safe`

Reference:
- `src/clean_bot_hardware/launch/robot_bringup.launch.py`
- `src/clean_bot_hardware/config/nav2_params.yaml` (see `controller_server` comments)

---

## 4) Safety + Arduino Bridge (clean_bot_hardware)

### 4.1 Emergency stop controller

- Node: `clean_bot_hardware/emergency_stop`
- Subscribes:
  - `/cmd_vel_nav` (Nav2 output)
  - `/cmd_vel` (optional other sources)
  - `/ultrasonic_range`
- Publishes:
  - `/cmd_vel_safe`
  - `/obstacle_detected`

Reference:
- `src/clean_bot_hardware/clean_bot_hardware/emergency_stop.py`

### 4.2 Arduino driver (motors + cleaning)

- Node: `clean_bot_hardware/arduino_driver`
- Subscribes:
  - `/cmd_vel_safe` (preferred)
  - `/cmd_vel_nav` (fallback when emergency stop node is disabled)
  - `/mission_command` (legacy cleaning commands only)
  - `/arduino_command` (cleaning relay/servo commands from mission controller)
- Publishes:
  - `/ultrasonic_range`
  - `/cmd_vel_debug` (debug)

Serial protocol behavior:

- Motion: converts Twist into PWM and sends `"pwm_left,pwm_right"`-style commands.
- Cleaning:
  - `start_clean` → sends `CLEAN_START`
  - `stop_clean` → sends `CLEAN_STOP`

Reference:
- `src/clean_bot_hardware/clean_bot_hardware/arduino_driver.py`

---

## 5) Mission Orchestration (clean_bot_mission)

The mission layer is responsible for:

- A user-facing state machine (`WAITING_FOR_SCAN → EXPLORING → WAITING_FOR_CLEAN → COVERAGE → RETURNING → COMPLETE`)
- Publishing control signals to exploration and coverage subsystems
- Triggering cleaning hardware (relay/servo) during cleaning

### 5.1 Mission commands and state

**Input** (external control):

- Topic: `/mission_command` (std_msgs/String)
- Examples:
  - `start_scan`, `stop_scan`
  - `start_clean`, `stop_clean`
  - `pause`, `resume`
  - `go_home`, `reset`

**Output** (mission status):

- Topic: `/mission_state` (std_msgs/String)
- Values: `WAITING_FOR_SCAN`, `EXPLORING`, ...

Reference:
- `src/clean_bot_mission/clean_bot_mission/full_mission.py`

### 5.2 Exploration control

Mission → Explorer control:

- Topic: `/exploration_control` (std_msgs/String)
- Values: `start`, `stop`, `pause`, `resume`, `reset`

Explorer → Mission completion signal:

- Topic: `/exploration_complete` (std_msgs/Bool)

Reference:
- `src/clean_bot_mission/clean_bot_mission/full_mission.py`
- `src/clean_bot_mission/clean_bot_mission/frontier_explorer.py`

### 5.3 Coverage control

Mission → Coverage control:

- Topic: `/coverage_control` (std_msgs/String)
- Values: `start`, `stop`, `pause`, `resume`, `reset`

Coverage → Mission completion signal:

- Topic: `/coverage_complete` (std_msgs/Bool)

Reference:
- `src/clean_bot_mission/clean_bot_mission/full_mission.py`
- `src/clean_bot_mission/clean_bot_mission/adaptive_coverage.py`

---

## 6) “Single-process” vs “Multi-node” mission runtime

This repo supports two ways to run the mission stack.

### Option A: Single-process (everything inside `full_mission`)

- You run: `ros2 run clean_bot_mission full_mission`
- The process creates:
  - `full_mission_controller` (mission state machine)
  - `frontier_explorer` (exploration)
  - `adaptive_coverage_planner` (coverage)

### Option B: Multi-node (recommended for launch files)

- You run a launch file that starts:
  - `full_mission` (controller only)
  - `frontier_explorer` (separate process)
  - `adaptive_coverage` (separate process)

This is controlled by:

- Parameter on `full_mission`: `run_subnodes`
  - `true` (default): create explorer + coverage nodes in-process
  - `false`: controller only; expect separate nodes from launch

Reference:
- `src/clean_bot_mission/clean_bot_mission/full_mission.py`

---

## 7) End-to-end Data Flow (Command → Motion)

### 7.1 Explore (start_scan → driving)

1. User sends `start_scan` to `/mission_command`.
2. `full_mission_controller`:
   - sets mission state to `EXPLORING`
   - publishes `"start"` on `/exploration_control`
3. `frontier_explorer`:
   - waits for a valid `/map` and TF `map → base_link`
   - detects frontiers on the occupancy grid
   - sends goals to Nav2 via the `navigate_to_pose` action
4. Nav2:
   - plans and tracks to goals
   - outputs velocity on `/cmd_vel_nav`
5. Safety layer (optional):
   - `emergency_stop_controller` filters `/cmd_vel_nav` → `/cmd_vel_safe`
6. Arduino:
   - `arduino_driver` consumes `/cmd_vel_safe` (or `/cmd_vel_nav` fallback)
   - converts Twist to PWM and writes serial

### 7.2 Clean (start_clean → relay/servo + driving)

1. User sends `start_clean` to `/mission_command`.
2. `full_mission_controller`:
   - activates cleaning hardware by publishing `start_clean` on `/arduino_command`
   - publishes `start` on `/coverage_control`
   - publishes `True` on `/exploration_complete` (as a trigger for coverage)
3. `adaptive_coverage_planner`:
   - reads `/map`
   - generates coverage waypoints
   - either:
     - sends Nav2 goals (`use_direct_drive:=false`), or
     - publishes Twist on `/cmd_vel_nav` (`use_direct_drive:=true`)
4. Nav2 + safety + Arduino chain is the same as exploration.

---

## 8) TF Tree (Where frames come from)

Core frames:

- `base_link`: robot body frame
- `laser`: LiDAR frame (from URDF via `robot_state_publisher`)
- `odom`: local odometry frame
- `map`: global SLAM frame

Producers:

- `robot_state_publisher`: static transforms (`base_link` → sensors)
- `rf2o_laser_odometry`: TF `odom → base_link`
- `cartographer_node`: TF `map → odom`

Net result:

- TF `map → base_link` exists via composition: `map→odom` + `odom→base_link`

This is the transform Nav2 and the explorer use to reason about robot pose.

---

## 9) Common “Nothing Moves / Arduino Gets Nothing” Causes

If `start_scan` is received but the robot does not move:

1. **No velocity published**
   - Check: `ros2 topic hz /cmd_vel_nav`
   - If 0 Hz, explorer may not be sending goals or Nav2 may be inactive.

2. **Explorer has no map yet**
   - Check: `ros2 topic echo /map --once`
   - Explorer requires `/map` and TF `map→base_link`.

3. **Nav2 can’t compute paths**
   - Look for Nav2 warnings about TF, costmaps, or controller server.

4. **Emergency stop always stopping**
   - Check `/ultrasonic_range` and `/obstacle_detected`.
   - If obstacles are always detected, `/cmd_vel_safe` may be near zero.

5. **Arduino serial not connected**
   - `arduino_driver` logs `✅ Connected to Arduino on ...` when OK.

6. **Command delivered too early (DDS discovery race)**
   - Single-shot publishes can be missed right after startup.
   - Use repeated publish, or `mission_cmd` helper.

Reference helper:
- `src/clean_bot_mission/clean_bot_mission/mission_cmd.py`

---

## 10) Debug Checklist (Fast)

Run these while the robot is up:

- Nodes:
  - `ros2 node list | grep -E 'full_mission|frontier_explorer|adaptive_coverage|arduino_driver|emergency_stop|cartographer|rf2o'`
- Mission input/output:
  - `ros2 topic info /mission_command`
  - `ros2 topic echo /mission_state --once`
- Velocity chain:
  - `ros2 topic hz /cmd_vel_nav`
  - `ros2 topic hz /cmd_vel_safe`
- TF sanity:
  - `ros2 run tf2_ros tf2_echo map base_link`

---

## 11) File Map (Where to read/modify behavior)

- Hardware bringup: `src/clean_bot_hardware/launch/robot_bringup.launch.py`
- Odometry: `src/clean_bot_hardware/launch/odom.launch.py`
- SLAM: `src/clean_bot_hardware/launch/cartographer.launch.py` and `src/clean_bot_hardware/config/cartographer.lua`
- Nav2 params: `src/clean_bot_hardware/config/nav2_params.yaml`
- Arduino bridge: `src/clean_bot_hardware/clean_bot_hardware/arduino_driver.py`
- Safety filter: `src/clean_bot_hardware/clean_bot_hardware/emergency_stop.py`
- Mission controller: `src/clean_bot_mission/clean_bot_mission/full_mission.py`
- Exploration: `src/clean_bot_mission/clean_bot_mission/frontier_explorer.py`
- Coverage: `src/clean_bot_mission/clean_bot_mission/adaptive_coverage.py`
- PC control bridge: `src/clean_bot_mission/scripts/telegram_bridge.py`

---

## 12) Notes on Multi-machine ROS graphs

If you run the mission on the Pi and the Telegram bridge on a PC:

- Both machines must share the same `ROS_DOMAIN_ID`.
- Duplicate node names (e.g., `frontier_explorer`) often mean the node is running twice on two hosts.
- If you see “nodes in graph share an exact name”, verify by checking processes on each host.
