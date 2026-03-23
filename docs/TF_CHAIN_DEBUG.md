# TF Chain Debug (Clean Bot)

This project depends heavily on TF being **continuous**, **non-conflicting**, and **complete**.
When TF is missing (or two nodes publish the same transform), RViz and Nav2 often look like they “randomly crash”, but the root cause is usually a single broken link in the TF chain.

## 1) The TF Tree We Expect

Nav2 (and RViz RobotModel) expects this chain to exist at all times:

- `map → odom`  (global SLAM correction)
- `odom → base_link` (local motion estimate)
- `base_link → {laser, imu_link, ultrasonic_link, wheels...}` (URDF fixed joints)

If **any** of these links is missing, RViz will show errors like:
- “No transform from [base_link] …” for many robot links
- LaserScan displayed but robot model is red / frame errors

### Who publishes what (in this repo)

- `robot_state_publisher`
  - Publishes: `base_link → laser`, `base_link → imu_link`, `base_link → ultrasonic_link`, wheels, etc.
  - Source: URDF/Xacro in `clean_bot_description`

- Cartographer (`cartographer_ros/cartographer_node`)
  - Publishes: `map → odom`
  - **Can also publish:** `odom → base_link` if `provide_odom_frame = true`

- rf2o (`rf2o_laser_odometry_node`) (optional)
  - Publishes: `odom → base_link` and `/odom` topic
  - **If rf2o is enabled, Cartographer must NOT also publish `odom → base_link`**

- `clean_bot_hardware/tf_odom_publisher`
  - Publishes: `/odom` (nav_msgs/Odometry)
  - Source: TF lookup of `odom → base_link`
  - Purpose: Nav2 and mission monitors expect an `/odom` topic even when rf2o is disabled.

## 2) The Most Common Failure Modes

### A) Missing `odom → base_link`
Symptoms:
- RViz RobotModel turns red (no transforms for most links)
- Nav2 costmaps complain about TF

Common cause in this workspace:
- rf2o disabled, but Cartographer is also configured with `provide_odom_frame = false`.

Fix:
- Ensure Cartographer config publishes local odom TF:
  - In `clean_bot_hardware/config/cartographer.lua`:
    - `published_frame = "base_link"`
    - `odom_frame = "odom"`
    - `provide_odom_frame = true`

### B) Two publishers for the same TF
Symptoms:
- TF appears, then “jumps” / becomes unstable
- Intermittent Nav2 failures

Common causes:
- rf2o publishes `odom→base_link` while Cartographer also publishes `odom→base_link`.

Fix:
- Only **one** node should publish a given transform.

### C) Time / clock jumps (TF extrapolation)
Symptoms:
- Errors like “extrapolation into the past/future”
- Works after restart; fails after a time sync event

Fix:
- Ensure system time is stable before launching ROS
- Avoid large NTP step adjustments during runtime (prefer slew)

### D) Cartographer not running / crashed
Symptoms:
- `map→odom` disappears
- RViz fixed frame `map` becomes invalid

Fix:
- Check Cartographer logs and config includes

## 3) Quick Checks (Commands)

These help confirm which link is missing.

- See the TF tree:
  - `ros2 run tf2_tools view_frames`

- Check if a transform exists right now:
  - `ros2 run tf2_ros tf2_echo odom base_link`
  - `ros2 run tf2_ros tf2_echo map odom`

- Confirm `/odom` exists (Nav2 expects it):
  - `ros2 topic echo /odom --once`

## 4) Repo-Specific Notes

- `clean_bot_hardware/launch/robot_bringup.launch.py`
  - rf2o is **optional** via `use_rf2o` (default: `false`).
  - When rf2o is off, `/odom` is published by `tf_odom_publisher`.

- `clean_bot_hardware/launch/slam.launch.py`
  - Must not disable IMU odom broadcaster by default (otherwise no local TF).

## 5) What “TF Crash” Usually Means

RViz itself rarely crashes due to TF; rather:
- RViz can’t render the robot model because the fixed frame can’t reach `base_link`.
- Nav2 nodes may stop planning because costmaps can’t transform sensor data.

If you capture logs, the fastest path is:
1) Identify the first missing edge in the chain (`map→odom` or `odom→base_link`).
2) Identify who was supposed to publish it.
3) Ensure there is exactly one publisher for that edge.
