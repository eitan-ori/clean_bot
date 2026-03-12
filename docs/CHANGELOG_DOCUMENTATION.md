# Documentation Updates

All documentation changes made during the cleanup (`cleanup/streamline-project` branch).

---

## README Files

### `README.md` (root) — Complete rewrite
- **Before:** Referenced Gazebo simulation, encoder odometry, EKF fusion, and multiple cleaning modes. File structure listed wrong filenames.
- **After:** Focused entirely on the physical robot. Includes:
  - Architecture diagram (Telegram PC ↔ ROS2 ↔ Raspberry Pi)
  - TF tree diagram
  - Hardware table (Pi, Arduino, RPLidar, IMU, Ultrasonic)
  - Prerequisites for Pi and PC
  - Quick start (3-step: launch robot, start Telegram, send commands)
  - Complete Telegram commands table (12 commands)
  - Key ROS2 topics table (including `/cmd_vel_nav`, `/scan_throttled`)
  - Package descriptions
  - Troubleshooting section
  - Accurate file structure tree

### `clean_bot_description/README.md` — Created (new file)
- Documents the URDF model package: chassis, wheels, sensor frames
- Notes that `<gazebo>` tags are retained for optional simulation but ignored on physical robot

### `clean_bot_hardware/README.md` — Rewritten
- **Removed:** EKF references, encoder odometry, `imu_odom_broadcaster` as primary, `coverage_mission` entry point, `robot-localization` dependency
- **Added:** Velocity safety chain section (`Nav2 → cmd_vel_nav → emergency_stop → cmd_vel → arduino`), `scan_throttler.py`, accurate package structure, correct topics table

### `clean_bot_mission/README.md` — Updated
- Added `scripts/` directory (telegram_bridge) to file structure
- Added RETURNING state to mission flow diagram
- Added note about auto-send map on completion

### `clean_bot_mission/scripts/TELEGRAM_README.md` — Updated
- Updated token setup instructions
- Removed hardcoded token references

---

## Technical Documentation (docs/)

### `docs/HOW_IT_WORKS.md`
- Updated odometry section: rf2o laser odometry (not wheel encoders)
- Removed EKF fusion references
- Updated sensor pipeline description

### `docs/Clean_Bot_Flow_Diagrams.md`
- Removed Gazebo-related flow nodes
- Updated to reflect single cleaning mode

### `docs/Clean_Bot_Learning_Guide.md`
- Added note: "ROS2 Humble project designed for real hardware"
- Updated file references to current names
- Removed simulation learning paths

### `docs/LAUNCH_PROCESS_FLOW.md`
- Fixed mission commands table: removed `start_clean` (random walk) and `start_clean_coverage` split
- Now shows single `start_clean` command for map-based adaptive coverage

---

## URDF File Comments

| File | Change |
|------|--------|
| `robot_core.xacro` | Clarified `<gazebo>` material tags are for optional simulation, ignored on physical robot |
| `lidar.xacro` | Updated header: physical RPLidar driver publishes `/scan` directly; Gazebo plugin is simulation-only |
| `imu.xacro` | Updated header: physical Grove IMU driver publishes to `/imu` topics; Gazebo plugin is simulation-only |
| `ultrasonic.xacro` | Added comment: Gazebo plugin ignored on physical robot |
| `inertial_macros.xacro` | Changed "realistic physical simulation in Gazebo" → "accurate physics calculations in the robot model" |

---

## Launch File Comments

| File | Change |
|------|--------|
| `robot_bringup.launch.py` | Translated Hebrew comments to English (throttler, rf2o frequency) |
| `robot_bringup.launch.py` | Fixed "Gazebo" comment to "physical robot" |
| `slam.launch.py` | Changed `Use simulation/Gazebo clock` → `Use simulated clock (always false for physical robot)` |
| `odom.launch.py` | Translated all Hebrew parameter comments to English |

---

## Inline Comment Translations (Hebrew → English)

All Hebrew comments were translated to English across:
- `clean_bot_mission/clean_bot_mission/simple_coverage.py` — Docstrings, class description, parameter comments
- `clean_bot_hardware/launch/odom.launch.py` — All parameter annotations
- `clean_bot_hardware/launch/robot_bringup.launch.py` — Throttler and rf2o comments
