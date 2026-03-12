# 🚀 Clean Bot Launch Process Flow

This document explains exactly what happens when you run:
```bash
ros2 launch clean_bot_mission cleaning_mission.launch.py
```

---

## 📊 High-Level Architecture

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                            LAUNCH HIERARCHY                                      │
├─────────────────────────────────────────────────────────────────────────────────┤
│                                                                                  │
│   cleaning_mission.launch.py                                                     │
│   ├── robot_bringup.launch.py (included)                                        │
│   │   ├── robot_state_publisher     ─── URDF/TF                                 │
│   │   ├── arduino_driver            ─── Motors + Ultrasonic                     │
│   │   ├── sllidar_node              ─── LiDAR Scanner                           │
│   │   ├── imu_publisher             ─── IMU Sensor                              │
│   │   ├── imu_filter_madgwick       ─── Orientation Filter                      │
│   │   ├── low_obstacle_detector     ─── Ultrasonic → PointCloud                 │
│   │   ├── emergency_stop            ─── Safety Velocity Filter                  │
│   │   ├── scan_throttler            ─── Reduce Scan Rate                        │
│   │   ├── rf2o_laser_odometry       ─── Laser-based Odometry                    │
│   │   ├── slam_toolbox              ─── Mapping & Localization                  │
│   │   └── nav2_bringup              ─── Navigation Stack                        │
│   │                                                                              │
│   └── full_mission_controller       ─── Mission Orchestration                   │
│                                                                                  │
└─────────────────────────────────────────────────────────────────────────────────┘
```

---

## 📁 Files Executed (In Order)

### **LAYER 1: Launch Files**

| Order | File | Location | Purpose |
|-------|------|----------|---------|
| 1 | `cleaning_mission.launch.py` | `clean_bot_mission/launch/` | **Entry point** - Orchestrates everything |
| 2 | `robot_bringup.launch.py` | `clean_bot_hardware/launch/` | **Hardware master** - Starts all drivers |
| 3 | `navigation_launch.py` | `nav2_bringup/launch/` | Nav2 navigation stack (external package) |

---

### **LAYER 2: Robot Description**

| Node | Package | Executable | Source Files | Purpose |
|------|---------|------------|--------------|---------|
| `robot_state_publisher` | robot_state_publisher | robot_state_publisher | URDF files below | Broadcasts robot structure to TF tree |

**URDF Files Loaded (via Xacro):**

| File | Location | Purpose |
|------|----------|---------|
| `robot.urdf.xacro` | `clean_bot_description/urdf/` | Main robot assembly file |
| `robot_core.xacro` | `clean_bot_description/urdf/` | Chassis, wheels, caster definition |
| `lidar.xacro` | `clean_bot_description/urdf/` | LiDAR mount and frame |
| `imu.xacro` | `clean_bot_description/urdf/` | IMU mount and frame |
| `ultrasonic.xacro` | `clean_bot_description/urdf/` | Ultrasonic sensor frame |
| `inertial_macros.xacro` | `clean_bot_description/urdf/` | Physics calculation helpers |

---

### **LAYER 3: Hardware Drivers**

| Node Name | Package | Executable | Source File | Purpose |
|-----------|---------|------------|-------------|---------|
| `arduino_driver` | clean_bot_hardware | arduino_driver | `arduino_driver.py` | Motor control, ultrasonic reading, cleaning relay/servo |
| `sllidar_node` | sllidar_ros2 | sllidar_node | External package | 360° laser scanner driver |
| `imu_publisher` | clean_bot_hardware | imu_publisher | `imu_publisher_node.py` | IMU I2C communication + ROS publisher |

**Hardware Files Detail:**

```
clean_bot_hardware/clean_bot_hardware/
├── arduino_driver.py       ← ⭐ CRITICAL: Controls motors via serial
│                              Subscribes: /cmd_vel
│                              Publishes: /ultrasonic_range
│                              Listens: /mission_command (for relay/servo)
│
├── imu_publisher_node.py   ← Reads IMU via I2C
│                              Publishes: /imu/data_raw, /imu/mag
│
└── simple_imu_driver.py    ← Low-level I2C driver (used by imu_publisher_node.py)
```

---

### **LAYER 4: Sensor Processing**

| Node Name | Package | Executable | Source File | Purpose |
|-----------|---------|------------|-------------|---------|
| `imu_filter` | imu_filter_madgwick | imu_filter_madgwick_node | External package | Fuses IMU data → stable orientation |
| `low_obstacle_detector` | clean_bot_hardware | low_obstacle_detector | `low_obstacle_detector.py` | Converts ultrasonic range → PointCloud2 |
| `scan_throttler` | topic_tools | throttle | External package | Reduces /scan rate from 10Hz → 5Hz |

**Sensor Processing Files Detail:**

```
clean_bot_hardware/clean_bot_hardware/
├── low_obstacle_detector.py  ← Converts ultrasonic to 3D points
│                                Subscribes: /ultrasonic_range
│                                Publishes: /low_obstacles (PointCloud2)
│                                Purpose: Detect floor-level obstacles (cables, etc.)
```

---

### **LAYER 5: Safety**

| Node Name | Package | Executable | Source File | Purpose |
|-----------|---------|------------|-------------|---------|
| `emergency_stop_controller` | clean_bot_hardware | emergency_stop | `emergency_stop.py` | Velocity filter for collision avoidance |

**Safety Flow:**
```
/cmd_vel_nav (from Nav2) → emergency_stop_controller → /cmd_vel (to Arduino)
                                    ↑
                          /ultrasonic_range
                          (if < 10cm → STOP)
                          (if < 30cm → SLOW DOWN)
```

---

### **LAYER 6: Odometry & Localization**

| Node Name | Package | Executable | Config File | Purpose |
|-----------|---------|------------|-------------|---------|
| `rf2o_laser_odometry` | rf2o_laser_odometry | rf2o_laser_odometry_node | - | Laser scan matching → odom frame |
| `slam_toolbox` | slam_toolbox | async_slam_toolbox_node | `mapper_params_online_async.yaml` | Builds map + publishes map→odom TF |

**Odometry Flow:**
```
/scan_throttled → rf2o_laser_odometry → /odom + TF(odom→base_link)
                                              ↓
                          slam_toolbox ← /scan
                               ↓
                     /map + TF(map→odom)
```

**Config File:**
```
clean_bot_hardware/config/
└── mapper_params_online_async.yaml  ← SLAM parameters (resolution, frames, etc.)
```

---

### **LAYER 7: Navigation (Nav2)**

| Component | Purpose |
|-----------|---------|
| `bt_navigator` | Behavior Tree executor for navigation |
| `planner_server` | Global path planning (NavFn/Smac) |
| `controller_server` | Local trajectory following (DWB/MPPI) |
| `costmap_2d` (global) | Static obstacle map from /map |
| `costmap_2d` (local) | Dynamic obstacles from /scan + /low_obstacles |
| `behavior_server` | Recovery behaviors (spin, backup) |
| `velocity_smoother` | Smooths velocity commands |

**Config File:**
```
clean_bot_hardware/config/
└── nav2_params.yaml  ← All Nav2 parameters (speeds, costmaps, planners)
```

---

### **LAYER 8: Mission Control**

| Node Name | Package | Executable | Source File | Purpose |
|-----------|---------|------------|-------------|---------|
| `full_mission_controller` | clean_bot_mission | full_mission | `full_mission.py` | State machine orchestrating entire mission |

**Mission Controller Dependencies:**
```
clean_bot_mission/clean_bot_mission/
├── full_mission.py           ← ⭐ MAIN MISSION STATE MACHINE
│                                States: WAITING_FOR_SCAN → EXPLORING → 
│                                        WAITING_FOR_CLEAN → COVERAGE → 
│                                        RETURNING → COMPLETE
│
├── frontier_explorer.py      ← Exploration algorithm (finds unexplored areas)
│                                Used during EXPLORING state
│
├── adaptive_coverage.py      ← Coverage path planning (zigzag patterns)
│                                Used during COVERAGE state
│
└── simple_coverage.py        ← Simple lawnmower pattern (backup)
```

### **Telegram Bridge (External Control)**

| Script | Package | Location | Purpose |
|--------|---------|----------|---------|
| `telegram_bridge.py` | clean_bot_mission | `scripts/` | Telegram bot for remote control (runs on PC) |

**Telegram Flow:**
```
Telegram User → telegram_bridge.py → /mission_command → full_mission_controller
full_mission_controller → /mission_state → telegram_bridge.py → Telegram User
```

---

## 🔄 Data Flow Diagram

```
┌──────────────────────────────────────────────────────────────────────────────┐
│                              DATA FLOW                                        │
└──────────────────────────────────────────────────────────────────────────────┘

                    ┌─────────────┐
                    │   Arduino   │
                    │  (Motors)   │
                    └──────▲──────┘
                           │ Serial
                           │
┌──────────────────────────┴────────────────────────────────────────────────┐
│                                                                            │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐                    │
│  │   LiDAR     │    │     IMU     │    │ Ultrasonic  │     SENSORS        │
│  │ sllidar_node│    │imu_publisher│    │(via Arduino)│                    │
│  └──────┬──────┘    └──────┬──────┘    └──────┬──────┘                    │
│         │                  │                  │                            │
│         ▼                  ▼                  ▼                            │
│      /scan           /imu/data_raw     /ultrasonic_range                  │
│         │                  │                  │                            │
│         ▼                  ▼                  ├────────────────────┐       │
│  ┌────────────┐     ┌────────────┐    ┌──────▼──────┐    ┌────────▼─────┐│
│  │   Throttle │     │  Madgwick  │    │ Low Obstacle │    │ Emergency   ││
│  │  (5 Hz)    │     │  Filter    │    │   Detector   │    │    Stop     ││
│  └──────┬─────┘     └──────┬─────┘    └──────┬───────┘    └────────┬────┘│
│         │                  │                  │                     │     │
│         ▼                  ▼                  ▼                     │     │
│  /scan_throttled      /imu/data        /low_obstacles              │     │
│         │                                     │                     │     │
│         ▼                                     │                     │     │
│  ┌────────────┐                              │                     │     │
│  │    RF2O    │──────► /odom                 │                     │     │
│  │  Odometry  │        TF(odom→base_link)    │                     │     │
│  └──────┬─────┘                              │                     │     │
│         │                                     │                     │     │
│         ▼                                     ▼                     │     │
│  ┌────────────┐                       ┌────────────┐               │     │
│  │   SLAM     │──────► /map           │   Nav2     │◄──────────────┘     │
│  │  Toolbox   │        TF(map→odom)   │  Costmaps  │                     │
│  └────────────┘                       └──────┬─────┘                     │
│                                              │                            │
│                                              ▼                            │
│                                       ┌────────────┐                     │
│                                       │   Nav2     │                     │
│                                       │  Planner   │                     │
│                                       └──────┬─────┘                     │
│                                              │                            │
│                                              ▼                            │
│                                       /cmd_vel_nav                        │
│                                              │                            │
│                        ┌─────────────────────┘                           │
│                        ▼                                                  │
│                 ┌────────────┐                                            │
│                 │ Emergency  │                                            │
│                 │   Stop     │                                            │
│                 └──────┬─────┘                                            │
│                        │                                                  │
│                        ▼                                                  │
│                    /cmd_vel ──────────► arduino_driver ──────► Motors    │
│                                                                           │
└───────────────────────────────────────────────────────────────────────────┘

                              MISSION CONTROL
                    ┌──────────────────────────────┐
                    │    full_mission_controller   │
                    │                              │
                    │  /mission_command ◄─── Telegram Bot (PC)
                    │          │                   │
                    │          ▼                   │
                    │  State Machine:              │
                    │  WAITING_FOR_SCAN            │
                    │       ↓ (start_scan)         │
                    │  EXPLORING                   │
                    │       ↓ (complete/stop_scan) │
                    │  WAITING_FOR_CLEAN           │
                    │       ↓ (start_clean)        │
                    │  COVERAGE                    │
                    │       ↓ (complete)           │
                    │  RETURNING                   │
                    │       ↓                      │
                    │  COMPLETE                    │
                    │                              │
                    │  /mission_state ────► Telegram Bot
                    └──────────────────────────────┘
```

---

## 🔧 Config Files Summary

| File | Location | Used By | Purpose |
|------|----------|---------|---------|
| `mapper_params_online_async.yaml` | `clean_bot_hardware/config/` | slam_toolbox | SLAM parameters |
| `nav2_params.yaml` | `clean_bot_hardware/config/` | Nav2 | Navigation tuning |

---

## 🎮 Control Interface

Commands are sent via `/mission_command` topic (String):

| Command | Action |
|---------|--------|
| `start_scan` | Begin exploration phase |
| `stop_scan` | Stop exploration, wait for clean command |
| `start_clean` | Start cleaning (random walk mode) |
| `start_clean_coverage` | Start cleaning (map-based coverage) |
| `stop_clean` | Stop cleaning |
| `go_home` | Return to starting position |
| `pause` | Pause current operation |
| `resume` | Resume from pause |
| `reset` | Reset to initial state |

**Example:**
```bash
ros2 topic pub --once /mission_command std_msgs/msg/String "data: 'start_scan'"
```

---

## 📝 Complete File List

```
EXECUTED CODE FILES:
====================

Launch Files:
├── clean_bot_mission/launch/cleaning_mission.launch.py     ← Entry point
└── clean_bot_hardware/launch/robot_bringup.launch.py       ← Hardware setup

Python Nodes (clean_bot_hardware):
├── clean_bot_hardware/arduino_driver.py                    ← Motors, ultrasonic
├── clean_bot_hardware/imu_publisher_node.py                ← IMU publisher  
├── clean_bot_hardware/simple_imu_driver.py                 ← IMU I2C driver
├── clean_bot_hardware/low_obstacle_detector.py             ← Ultrasonic → cloud
└── clean_bot_hardware/emergency_stop.py                    ← Safety filter

Python Nodes (clean_bot_mission):
├── clean_bot_mission/full_mission.py                       ← State machine
├── clean_bot_mission/frontier_explorer.py                  ← Exploration
└── clean_bot_mission/adaptive_coverage.py                  ← Coverage paths

URDF Files (loaded by robot_state_publisher):
├── clean_bot_description/urdf/robot.urdf.xacro             ← Main assembly
├── clean_bot_description/urdf/robot_core.xacro             ← Body structure
├── clean_bot_description/urdf/lidar.xacro                  ← LiDAR frame
├── clean_bot_description/urdf/imu.xacro                    ← IMU frame
├── clean_bot_description/urdf/ultrasonic.xacro             ← Ultrasonic frame
└── clean_bot_description/urdf/inertial_macros.xacro        ← Physics helpers

Config Files:
├── clean_bot_hardware/config/mapper_params_online_async.yaml ← SLAM config
└── clean_bot_hardware/config/nav2_params.yaml                ← Nav2 config

External Packages (system-installed):
├── sllidar_ros2                                            ← LiDAR driver
├── rf2o_laser_odometry                                     ← Laser odometry
├── imu_filter_madgwick                                     ← IMU filter
├── slam_toolbox                                            ← SLAM
├── nav2_bringup/navigation_launch.py                       ← Navigation
├── robot_state_publisher                                   ← TF publisher
└── topic_tools (throttle)                                  ← Rate limiter
```

---

## ⚠️ Common Issues

1. **"map frame doesn't exist"** → SLAM Toolbox not publishing TF (check `transform_publish_period` parameter)
2. **Robot doesn't move** → Check `/cmd_vel` echo, verify emergency_stop isn't blocking
3. **LiDAR not working** → Check `/dev/ttyUSB0` permissions (`sudo chmod 666 /dev/ttyUSB0`)
4. **Arduino not connecting** → Check `/dev/ttyACM0` exists and permissions
5. **Nav2 crashes** → Check config file paths and parameter validity
