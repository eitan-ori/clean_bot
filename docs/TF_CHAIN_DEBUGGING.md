# TF Chain Debugging (Clean Bot)

This project’s navigation and mapping depend on a *consistent, non-duplicated* TF tree. Most “TF crashes” in RViz/Nav2 are one of:

- Missing a required transform (most often `odom → base_link`)
- Two different nodes publishing the *same* transform (TF authority conflict)
- Time jumping backwards/forwards (timestamps don’t line up; TF extrapolation errors)

This document maps the intended TF chain in this repo, explains the common breakpoints, and gives quick checks.

## 1) Intended TF Tree (Physical Robot)

### Core frames

- `map` — global frame (SLAM)
- `odom` — locally-continuous frame (local SLAM / odom)
- `base_link` — robot body frame

### Sensor frames (examples)

These are typically published from URDF via `robot_state_publisher`:

- `base_link → laser`
- `base_link → imu_link`
- `base_link → ultrasonic_link`

## 2) Who Publishes Which Transform

### A) Robot description transforms

Publisher: `robot_state_publisher`

- Publishes the robot’s static kinematic tree from URDF.
- If `base_link → laser` (or `imu_link`) is missing, Cartographer and RViz will fail to associate sensor data with the robot.

Launch location:
- `clean_bot_hardware/launch/robot_bringup.launch.py`

### B) SLAM transforms (Cartographer)

Publisher: `cartographer_ros/cartographer_node`

Two supported modes exist conceptually:

1) **No external odometry (recommended when rf2o is disabled)**
   - Cartographer provides `odom → base_link`
   - Cartographer provides `map → odom`

2) **External odometry (rf2o enabled)**
   - External odom provides `odom → base_link`
   - Cartographer provides `map → odom`

Config location:
- `clean_bot_hardware/config/cartographer.lua`

Launch location:
- `clean_bot_hardware/launch/cartographer.launch.py`

### C) External odometry (rf2o) (optional)

Publisher: `rf2o_laser_odometry/rf2o_laser_odometry_node`

- Publishes TF: `odom → base_link`
- Publishes topic: `/odom` (depending on node params)

Launch location:
- `clean_bot_hardware/launch/odom.launch.py`
- Included from `clean_bot_hardware/launch/robot_bringup.launch.py` when `use_rf2o:=true`

### D) `/odom` topic publishing when rf2o is disabled

Publisher: `clean_bot_hardware/tf_odom_publisher`

- Reads TF(`odom → base_link`) and publishes a `nav_msgs/Odometry` message on `/odom`.
- This is for Nav2/monitors that expect an `/odom` topic.

Launch location:
- `clean_bot_hardware/launch/robot_bringup.launch.py` when `use_rf2o:=false`

## 3) The Most Common “TF Crash” in This Repo

### Symptom

RViz RobotModel shows lots of:

- “No transform from [base_link] …”
- “No transform from [imu_link] …”

Nav2 / costmaps show:

- TF extrapolation errors
- Costmap dropping sensor messages

### Root cause (most common)

`odom → base_link` is missing.

In this repo, that happens when:

- `use_rf2o:=false` (so rf2o does **not** publish `odom → base_link`)
- Cartographer is configured to **not** provide `odom → base_link` (i.e. `provide_odom_frame=false`), leaving nobody to publish it.

### Fix

When rf2o is disabled, ensure Cartographer is configured to provide `odom → base_link`:

- `published_frame = "base_link"`
- `provide_odom_frame = true`

(These are guarded by unit tests in `clean_bot_hardware/test/test_tf_chain.py`.)

## 4) Another Common Failure: Duplicate TF Publishers

### Symptom

- TF warnings about repeated or conflicting transforms
- Intermittent “works then breaks” depending on startup order

### Root cause

Two nodes publish the same transform, typically `odom → base_link`.

Examples:

- rf2o publishes `odom → base_link` AND Cartographer is also configured with `provide_odom_frame=true`

### Fix

Pick exactly one source for `odom → base_link`:

- If using rf2o: set `provide_odom_frame=false` in Cartographer and enable `use_rf2o:=true`
- If not using rf2o: set `provide_odom_frame=true` in Cartographer and keep `use_rf2o:=false`

## 5) Time-Related TF Failures (Extrapolation)

### Symptom

- “Lookup would require extrapolation into the past/future”
- Errors are worse right after boot

### Root cause

Clock jumps (NTP step adjustments) can make TF buffers inconsistent.

### Mitigations

- Ensure system time is stable *before* launching the ROS stack (especially on the Pi).
- Keep goal/action stamps robust (this repo uses “latest/zero” stamping in mission nodes).

## 6) Quick Checklist (10 seconds)

- `ros2 run tf2_tools view_frames` (produces a PDF of the TF tree)
- `ros2 topic echo /tf --once` and `ros2 topic echo /tf_static --once`
- Confirm you have a continuous chain from your fixed frame to `base_link`.

Practical target chain for Nav2:

- `map → odom` (Cartographer)
- `odom → base_link` (Cartographer *or* rf2o, but not both)
- `base_link → laser` (robot_state_publisher)
