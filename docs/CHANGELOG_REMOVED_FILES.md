# Removed Files and Packages

All files and packages removed during the project cleanup (`cleanup/streamline-project` branch).

---

## Removed Packages

### `ARCHIVE/` — Gazebo simulation package
- **What:** Complete Gazebo simulation environment (`clean_bot_gazebo`, `clean_bot_navigation`, `my_robot_slam`) with worlds, URDF, launch files, Arduino bridge scripts, tests, and configs.
- **Why:** The project is a physical robot. Gazebo simulation is no longer needed and was not maintained.
- **Files:** ~47 files, ~4,500+ lines removed.

### `jsk_rqt_plugins/`, `jsk_rviz_plugins/`, `jsk_visualization/`
- **What:** JSK visualization toolkit submodules (contained only `COLCON_IGNORE` markers — empty packages).
- **Why:** Never used; empty placeholder directories with no functionality.

---

## Removed Source Files

| File | Reason |
|------|--------|
| `clean_bot_mission/clean_bot_mission/exploration.py` | Legacy exploration node, replaced by `frontier_explorer.py` |
| `clean_bot_mission/clean_bot_mission/mission.py` | Legacy mission controller, replaced by `full_mission.py` |
| `clean_bot_mission/clean_bot_mission/coverage_mission.py` | Legacy coverage mission, replaced by `adaptive_coverage.py` |
| `clean_bot_description/urdf/gazebo_control.xacro` | Gazebo differential drive plugin (not needed on physical robot) |
| `server.py` | Flask web server (duplicate/alternative to Telegram bridge) |
| `check_lidar.py` | Root-level duplicate of `clean_bot_hardware/scripts/check_lidar.py` |
| `frames_2026-01-05_00.21.14.gv` | Generated TF tree graph file |
| `frames_2026-01-05_00.21.14.pdf` | Generated TF tree PDF |

## Removed Config Files

| File | Reason |
|------|--------|
| `clean_bot_hardware/config/ekf.yaml` | EKF (robot_localization) config — not used; rf2o provides odometry directly |

## Removed Entry Points (setup.py)

These console_scripts entries were removed from `clean_bot_mission/setup.py`:

| Entry Point | Pointed To | Reason |
|-------------|-----------|--------|
| `mission_node` | `mission:main` | `mission.py` deleted |
| `exploration_node` | `exploration:main` | `exploration.py` deleted |
| `coverage_mission` | `coverage_mission:main` | `coverage_mission.py` deleted |

---

## Removed Code (within existing files)

### Random and Square Cleaning (~350 lines)
- **File:** `clean_bot_mission/clean_bot_mission/full_mission.py`
- Removed `RandomWalkState`, `SquareCleanState`, `start_random_walk()`, `start_square_clean()`, `random_walk_step()`, `square_walk_step()`, and all associated timer/state logic.

### `/coverage` Telegram command
- **File:** `clean_bot_mission/scripts/telegram_bridge.py`
- Removed separate `/coverage` handler and registration. The `/clean` command now always triggers map-based adaptive coverage.
