# Clean Bot Mission

Autonomous exploration and coverage (cleaning) missions for the Clean Bot robot.

## 🎯 Overview

This package provides a complete autonomous cleaning solution with **external control**:

1. **Wait for Scan Command**: Robot waits for `start_scan` command
2. **Exploration Phase**: Robot autonomously explores unknown environment using frontier-based exploration
3. **Wait for Clean Command**: After mapping (or on `stop_scan`), robot waits for `start_clean` command
4. **Coverage Phase**: Robot executes optimal coverage path to clean all free space
5. **Cycle Support**: Can stop cleaning and return to scanning, repeatedly

## 🎮 Mission Control Commands

Control the robot via the `/mission_command` topic:

```bash
# Start exploration/scanning
ros2 topic pub --once /mission_command std_msgs/msg/String "data: 'start_scan'"

# Stop scanning and wait for clean command
ros2 topic pub --once /mission_command std_msgs/msg/String "data: 'stop_scan'"

# Start cleaning/coverage
ros2 topic pub --once /mission_command std_msgs/msg/String "data: 'start_clean'"

# Stop cleaning
ros2 topic pub --once /mission_command std_msgs/msg/String "data: 'stop_clean'"

# Return to home position
ros2 topic pub --once /mission_command std_msgs/msg/String "data: 'go_home'"

# Reset to initial waiting state
ros2 topic pub --once /mission_command std_msgs/msg/String "data: 'reset'"

# Pause current operation
ros2 topic pub --once /mission_command std_msgs/msg/String "data: 'pause'"

# Resume from pause
ros2 topic pub --once /mission_command std_msgs/msg/String "data: 'resume'"
```

## 🔄 Mission State Flow

```
┌─────────────────────┐
│  WAITING_FOR_SCAN   │ ◀──────────────────────┐
└──────────┬──────────┘                        │
           │ start_scan                        │ reset
           ▼                                   │
┌─────────────────────┐                        │
│     EXPLORING       │ ◀──┐                   │
└──────────┬──────────┘    │                   │
           │               │ start_scan        │
           │ stop_scan or  │                   │
           │ auto-complete │                   │
           ▼               │                   │
┌─────────────────────┐    │                   │
│  WAITING_FOR_CLEAN  │────┘                   │
└──────────┬──────────┘                        │
           │ start_clean                       │
           ▼                                   │
┌─────────────────────┐                        │
│      COVERAGE       │────────────────────────┤
└──────────┬──────────┘ stop_clean             │
           │                                   │
           │ auto-complete                     │
           ▼                                   │
┌─────────────────────┐                        │
│      COMPLETE       │────────────────────────┘
└─────────────────────┘
```

## 🚀 Quick Start

### Full Autonomous Mission

```bash
# Build
cd ~/robot_ws
colcon build --packages-select clean_bot_mission
source install/setup.bash

# Run full mission (waits for commands)
ros2 launch clean_bot_mission cleaning_mission.launch.py

# Or with auto-start (starts scanning immediately)
ros2 run clean_bot_mission full_mission --ros-args -p auto_start:=true

# With custom coverage width (14cm for vacuum cleaner)
ros2 launch clean_bot_mission cleaning_mission.launch.py coverage_width:=0.14
```

### Run Components Separately

```bash
# First, launch hardware + SLAM + Nav2
ros2 launch clean_bot_hardware robot_bringup.launch.py

# Then in another terminal, run exploration only
ros2 run clean_bot_mission frontier_explorer

# Or run coverage only (needs existing map)
ros2 run clean_bot_mission adaptive_coverage
```

## 📦 Nodes

### 1. `frontier_explorer`
Autonomous exploration using frontier-based algorithm.

**How it works:**
- Detects "frontiers" = boundaries between known and unknown space
- Navigates to nearest frontier
- Repeats until map is complete (no more frontiers)

**Control (via `/exploration_control` topic):**
- `start` - Start/resume exploration
- `stop` - Stop exploration
- `pause` - Pause exploration
- `resume` - Resume from pause
- `reset` - Reset state

**Parameters:**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `min_frontier_size` | 5 | Minimum cells for valid frontier |
| `robot_radius` | 0.18 | Safety margin (meters) |
| `exploration_timeout` | 600.0 | Max exploration time (seconds) |
| `goal_tolerance` | 0.3 | How close to reach frontier |
| `auto_start` | true | Auto-start when map received |

**Topics Published:**
- `/exploration_complete` (std_msgs/Bool) - True when done
- `/exploration_state` (std_msgs/String) - Current state
- `/frontiers` (visualization_msgs/MarkerArray) - For RViz

---

### 2. `adaptive_coverage`
Coverage path planner that adapts to room shape.

**How it works:**
- Inflates obstacles for robot safety
- Decomposes free space into vertical strips
- Generates zigzag (boustrophedon) pattern within each strip
- Optimizes path order to minimize travel

**Control (via `/coverage_control` topic):**
- `start` - Start/resume coverage
- `stop` - Stop coverage
- `pause` - Pause coverage
- `resume` - Resume from pause
- `reset` - Reset state

**Parameters:**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `coverage_width` | 0.14 | Cleaning stripe width (meters) |
| `overlap_ratio` | 0.15 | Overlap between stripes (15%) |
| `robot_radius` | 0.18 | For obstacle inflation |
| `timeout_per_waypoint` | 90 | Seconds per goal |
| `start_on_exploration_complete` | true | Auto-start after exploration |

**Topics Subscribed:**
- `/map` (nav_msgs/OccupancyGrid) - Map from SLAM
- `/exploration_complete` (std_msgs/Bool) - Trigger
- `/coverage_control` (std_msgs/String) - Control commands

**Topics Published:**
- `/coverage_complete` (std_msgs/Bool)
- `/coverage_state` (std_msgs/String) - Current state
- `/coverage_path` (nav_msgs/Path) - For visualization
- `/coverage_waypoints` (visualization_msgs/MarkerArray)

---

### 3. `full_mission`
Combined mission controller (runs both exploration and coverage).

```bash
ros2 run clean_bot_mission full_mission --ros-args \
    -p coverage_width:=0.14 \
    -p auto_start:=false
```

**Parameters:**
| Parameter | Default | Description |
|-----------|---------|-------------|
| `coverage_width` | 0.14 | Cleaning stripe width (meters) |
| `auto_start` | false | Auto-start scan on launch |
| `return_home_after` | true | Return home when complete |
| `home_x` | 0.0 | Home X coordinate |
| `home_y` | 0.0 | Home Y coordinate |

**Topics:**
- `/mission_command` (std_msgs/String) - Control input
- `/mission_state` (std_msgs/String) - Current state output

## 📊 Topics Overview

```
                    ┌─────────────────┐
    /mission_cmd    │  full_mission   │    /mission_state
    ───────────────▶│   controller    │───────────────────▶
                    └────────┬────────┘
                             │ /exploration_control
                             │ /coverage_control
                             ▼
                    ┌─────────────────┐
                    │  frontier_      │
                    │  explorer       │
                    └────────┬────────┘
                             │ /exploration_complete
                             ▼
┌──────────┐    /map    ┌─────────────────┐    /cmd_vel    ┌──────────┐
│  SLAM    │───────────▶│   adaptive_     │───────────────▶│  Nav2    │
│ Toolbox  │            │   coverage      │                │          │
└──────────┘            └────────┬────────┘                └──────────┘
                                 │ /coverage_complete
                                 ▼
                    ┌─────────────────────┐
                    │  Mission Done!      │
                    │  (or cycle again)   │
                    └─────────────────────┘
```

## 🛠️ Algorithm Details

### Frontier-Based Exploration

1. **Frontier Detection**: Find cells that are FREE and adjacent to UNKNOWN
2. **Clustering**: Group adjacent frontier cells into regions
3. **Selection**: Choose best frontier based on:
   - Size (larger = more unexplored area)
   - Distance (closer = faster to reach)
4. **Navigation**: Use Nav2 to reach frontier
5. **Repeat**: Until no frontiers remain

### Adaptive Coverage Planning

1. **Map Processing**:
   - Inflate obstacles by robot radius
   - Identify free space regions

2. **Path Generation**:
   - Scan map in vertical strips (width = coverage_width)
   - For each strip, find free segments
   - Generate zigzag: bottom→top, then top→bottom
   - Skip occupied/unknown areas

3. **Path Optimization**:
   - Reorder waypoints using nearest-neighbor heuristic
   - Minimize total travel distance

## 🔧 Troubleshooting

### Robot doesn't explore
- Check SLAM is publishing `/map`
- Check Nav2 is active: `ros2 node list | grep nav2`
- Check frontiers are detected: view `/frontiers` in RViz

### Coverage path misses areas
- Increase `coverage_width` slightly
- Decrease `overlap_ratio` for more passes
- Check map quality (no holes in free space)

### Robot gets stuck
- Increase `robot_radius` for more safety margin
- Check costmap inflation in Nav2 params
- Reduce `max_linear_speed` in Nav2

## 📁 Files

```
clean_bot_mission/
├── clean_bot_mission/
│   ├── frontier_explorer.py    # Autonomous exploration
│   ├── adaptive_coverage.py    # Coverage path planning
│   ├── full_mission.py         # Combined controller
│   ├── coverage_mission.py     # Legacy simple coverage
│   ├── exploration.py          # Legacy waypoint exploration
│   └── mission.py              # Legacy mission
├── launch/
│   └── cleaning_mission.launch.py  # Main launch file
├── package.xml
└── setup.py
```

## 📜 License

MIT License - Clean Bot Team
