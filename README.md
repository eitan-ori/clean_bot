# Clean Bot - Autonomous Cleaning Robot

A ROS 2 Humble autonomous cleaning robot simulation featuring SLAM-based navigation, Gazebo simulation, and mission planning.

![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue)
![Gazebo](https://img.shields.io/badge/Gazebo-Classic-orange)
![Nav2](https://img.shields.io/badge/Nav2-Navigation-green)
![SLAM](https://img.shields.io/badge/SLAM-Toolbox-purple)

## Table of Contents

- [Overview](#overview)
- [Architecture](#architecture)
- [Packages](#packages)
  - [clean_bot_description](#clean_bot_description)
  - [clean_bot_gazebo](#clean_bot_gazebo)
  - [clean_bot_navigation](#clean_bot_navigation)
  - [clean_bot_mission](#clean_bot_mission)
- [Installation](#installation)
- [Quick Start](#quick-start)
- [Robot Specifications](#robot-specifications)
- [Configuration](#configuration)
- [Troubleshooting](#troubleshooting)

---

## Overview

Clean Bot is a differential-drive mobile robot designed for autonomous cleaning tasks. The robot uses:

- **SLAM Toolbox** for simultaneous localization and mapping
- **Nav2** for path planning and obstacle avoidance
- **Gazebo Classic** for realistic physics simulation
- **Python-based mission control** for waypoint navigation

The robot autonomously navigates through a room, following a predefined cleaning path, and returns to its starting position (dock).

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        ROS 2 System                              │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌──────────────┐     ┌──────────────┐     ┌──────────────┐     │
│  │   Gazebo     │────▶│  SLAM        │────▶│   Nav2       │     │
│  │  Simulation  │     │  Toolbox     │     │  Navigation  │     │
│  └──────────────┘     └──────────────┘     └──────────────┘     │
│         │                    │                    │              │
│         │                    │                    │              │
│         ▼                    ▼                    ▼              │
│    /scan, /odom         /map, /tf           /cmd_vel            │
│    /tf, /imu                                                     │
│                                                                  │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │                    Mission Node                           │   │
│  │              (nav2_simple_commander)                      │   │
│  └──────────────────────────────────────────────────────────┘   │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

### TF Tree

```
map
 └── odom (published by SLAM Toolbox)
      └── base_link (published by diff_drive plugin)
           ├── chassis
           │    ├── laser_frame
           │    └── imu_link
           ├── left_wheel
           ├── right_wheel
           └── caster_wheel
```

---

## Packages

### clean_bot_description

**Purpose**: Defines the robot's physical structure using URDF/Xacro files.

**Key Files**:
| File | Description |
|------|-------------|
| `urdf/robot.urdf.xacro` | Main robot description entry point |
| `urdf/robot_core.xacro` | Chassis, wheels, and caster definition |
| `urdf/lidar.xacro` | 2D LiDAR sensor (360° scanning) |
| `urdf/imu.xacro` | IMU sensor for orientation |
| `urdf/gazebo_control.xacro` | Differential drive controller plugin |
| `urdf/inertial_macros.xacro` | Helper macros for inertia calculations |
| `launch/rsp.launch.py` | Robot State Publisher launch file |

**Robot Structure**:
- White chassis box (0.3m × 0.3m × 0.15m)
- Two driven wheels (0.1m diameter)
- Front caster wheel for stability
- Top-mounted 2D LiDAR

---

### clean_bot_gazebo

**Purpose**: Provides Gazebo simulation environment and world files.

**Key Files**:
| File | Description |
|------|-------------|
| `launch/sim.launch.py` | Main simulation launch file |
| `worlds/room.world` | Indoor room environment (~5m × 5m) |

**World Features**:
- Four walls enclosing the room
- Dock station (red box) at position (2, 2)
- Flat ground plane
- Ambient lighting

**Launch Arguments**:
```bash
ros2 launch clean_bot_gazebo sim.launch.py gui:=true  # With Gazebo GUI (default)
ros2 launch clean_bot_gazebo sim.launch.py gui:=false # Headless mode
```

---

### clean_bot_navigation

**Purpose**: Configures Nav2 navigation stack and SLAM Toolbox.

**Key Files**:
| File | Description |
|------|-------------|
| `launch/navigation.launch.py` | Launches Nav2 + SLAM Toolbox |
| `config/nav2_params.yaml` | Nav2 configuration (planners, controllers, costmaps) |
| `config/mapper_params_online_async.yaml` | SLAM Toolbox configuration |

**Navigation Stack Components**:

| Component | Plugin/Type | Description |
|-----------|-------------|-------------|
| **Localization** | SLAM Toolbox | Online async mapping mode |
| **Global Planner** | NavFn | Dijkstra-based path planning |
| **Local Controller** | DWB | Dynamic Window approach |
| **Behaviors** | Spin, Backup, Wait | Recovery behaviors |
| **Costmaps** | Voxel + Inflation | Obstacle and inflation layers |

**Key Parameters**:
```yaml
# Velocity limits
max_vel_x: 0.26 m/s
max_vel_theta: 1.0 rad/s

# Costmap
resolution: 0.05 m
robot_radius: 0.22 m
inflation_radius: 0.55 m
```

---

### clean_bot_mission

**Purpose**: Python-based autonomous mission execution using Nav2 Simple Commander.

**Key Files**:
| File | Description |
|------|-------------|
| `clean_bot_mission/mission.py` | Main mission logic |
| `setup.py` | Package setup and entry points |

**Mission Flow**:
```
1. Initialize Navigator
        │
        ▼
2. Wait for Nav2 + SLAM active
        │
        ▼
3. Set initial pose at (0, 0)
        │
        ▼
4. Execute cleaning path:
   (0,0) → (1.5,0) → (1.5,1.5) → (0,1.5) → (0,0)
        │
        ▼
5. Report success/failure
```

**Entry Point**:
```bash
ros2 run clean_bot_mission mission_node
```

---

## Installation

### Prerequisites

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic (gazebo11)
- Nav2
- SLAM Toolbox

### Install Dependencies

```bash
# Install ROS 2 packages
sudo apt update
sudo apt install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher
```

### Build Workspace

```bash
cd ~/robot_ws
colcon build
source install/setup.bash
```

---

## Quick Start

### Option 1: Run All Components Separately (Recommended for Debugging)

**Terminal 1 - Simulation**:
```bash
cd ~/robot_ws
source install/setup.bash
ros2 launch clean_bot_gazebo sim.launch.py
```

**Terminal 2 - Navigation**:
```bash
cd ~/robot_ws
source install/setup.bash
ros2 launch clean_bot_navigation navigation.launch.py
```

**Terminal 3 - Mission**:
```bash
cd ~/robot_ws
source install/setup.bash
ros2 run clean_bot_mission mission_node
```

### Option 2: Manual Control

Use `teleop_twist_keyboard` to manually control the robot:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## Robot Specifications

### Physical Properties

| Property | Value |
|----------|-------|
| Chassis Size | 0.3m × 0.3m × 0.15m |
| Total Mass | ~0.5 kg |
| Wheel Diameter | 0.1 m |
| Wheel Separation | 0.35 m |
| Ground Clearance | 0.05 m |

### Sensors

| Sensor | Type | Topic | Frame | Update Rate |
|--------|------|-------|-------|-------------|
| LiDAR | 2D Laser | `/scan` | `laser_frame` | 10 Hz |
| IMU | 6-DOF | `/imu` | `imu_link` | 100 Hz |
| Odometry | Wheel Encoders | `/odom` | `odom` | 30 Hz |

### LiDAR Specifications

| Parameter | Value |
|-----------|-------|
| Horizontal FOV | 360° |
| Angular Resolution | 1° (360 samples) |
| Min Range | 0.3 m |
| Max Range | 12.0 m |
| Noise | Gaussian (σ = 0.01) |

### Motion Capabilities

| Parameter | Value |
|-----------|-------|
| Max Linear Velocity | 0.26 m/s |
| Max Angular Velocity | 1.0 rad/s |
| Max Linear Acceleration | 2.5 m/s² |
| Max Angular Acceleration | 3.2 rad/s² |
| Max Wheel Torque | 200 Nm |

---

## Configuration

### Customizing Navigation Parameters

Edit `clean_bot_navigation/config/nav2_params.yaml`:

```yaml
# Adjust velocity limits
controller_server:
  ros__parameters:
    FollowPath:
      max_vel_x: 0.26      # Increase for faster movement
      max_vel_theta: 1.0   # Increase for faster turning

# Adjust costmap settings
local_costmap:
  local_costmap:
    ros__parameters:
      robot_radius: 0.22   # Match your robot size
      inflation_radius: 0.55
```

### Customizing SLAM Parameters

Edit `clean_bot_navigation/config/mapper_params_online_async.yaml`:

```yaml
slam_toolbox:
  ros__parameters:
    resolution: 0.05           # Map resolution (m/cell)
    max_laser_range: 12.0      # Match your LiDAR range
    minimum_travel_distance: 0.5  # Min distance before updating map
    map_update_interval: 5.0   # Map update frequency (seconds)
```

### Customizing Mission Waypoints

Edit `clean_bot_mission/clean_bot_mission/mission.py`:

```python
# Define your custom waypoints (x, y, yaw)
waypoints = [
    (1.0, 0.0, 0.0),      # First waypoint
    (1.0, 1.0, 1.57),     # Second waypoint (facing up)
    (0.0, 1.0, 3.14),     # Third waypoint (facing left)
    (0.0, 0.0, -1.57),    # Return to start
]
```

---

## Topics Reference

### Published Topics

| Topic | Type | Publisher | Description |
|-------|------|-----------|-------------|
| `/scan` | `sensor_msgs/LaserScan` | Gazebo LiDAR | Laser scan data |
| `/odom` | `nav_msgs/Odometry` | Gazebo diff_drive | Wheel odometry |
| `/imu` | `sensor_msgs/Imu` | Gazebo IMU | IMU data |
| `/map` | `nav_msgs/OccupancyGrid` | SLAM Toolbox | Generated map |
| `/tf` | `tf2_msgs/TFMessage` | Multiple | Transform tree |

### Subscribed Topics

| Topic | Type | Subscriber | Description |
|-------|------|------------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Gazebo diff_drive | Velocity commands |
| `/goal_pose` | `geometry_msgs/PoseStamped` | Nav2 | Navigation goal |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/slam_toolbox/save_map` | `slam_toolbox/srv/SaveMap` | Save current map |
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Navigate to goal |

---

## Troubleshooting

### Common Issues

#### 1. "Failed to compute odom pose" error

**Cause**: SLAM Toolbox can't find the `odom → base_link` transform.

**Solution**: Ensure Gazebo simulation is running and publishing transforms:
```bash
ros2 run tf2_ros tf2_echo odom base_link
```

#### 2. "Robot is out of bounds of the costmap"

**Cause**: Initial costmap is small; this warning clears once SLAM builds the map.

**Solution**: Wait a few seconds for SLAM to initialize the map.

#### 3. Mission node hangs on "Waiting for Nav2"

**Cause**: Navigation stack not fully active.

**Solution**: Ensure navigation launch completed successfully:
```bash
ros2 node list | grep -E "bt_navigator|controller_server|planner_server"
```

#### 4. Robot doesn't move

**Cause**: No velocity commands being sent or blocked by costmap.

**Solution**: Check `/cmd_vel` topic:
```bash
ros2 topic echo /cmd_vel
```

### Useful Debug Commands

```bash
# Check all running nodes
ros2 node list

# Visualize TF tree
ros2 run tf2_tools view_frames

# Check topic frequencies
ros2 topic hz /scan
ros2 topic hz /odom

# Echo sensor data
ros2 topic echo /scan --once
ros2 topic echo /odom --once

# Check Nav2 status
ros2 service call /bt_navigator/get_state lifecycle_msgs/srv/GetState
```

---

## File Structure

```
robot_ws/src/
├── clean_bot_description/      # Robot URDF model
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── launch/
│   │   └── rsp.launch.py      # Robot State Publisher
│   └── urdf/
│       ├── robot.urdf.xacro   # Main entry point
│       ├── robot_core.xacro   # Chassis and wheels
│       ├── lidar.xacro        # LiDAR sensor
│       ├── imu.xacro          # IMU sensor
│       ├── gazebo_control.xacro # Diff drive plugin
│       └── inertial_macros.xacro
│
├── clean_bot_gazebo/           # Simulation
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── launch/
│   │   └── sim.launch.py      # Simulation launcher
│   └── worlds/
│       └── room.world         # Indoor environment
│
├── clean_bot_navigation/       # Navigation
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── launch/
│   │   └── navigation.launch.py
│   └── config/
│       ├── nav2_params.yaml              # Nav2 configuration
│       └── mapper_params_online_async.yaml # SLAM config
│
└── clean_bot_mission/          # Mission control
    ├── setup.py
    ├── package.xml
    ├── resource/
    └── clean_bot_mission/
        ├── __init__.py
        └── mission.py         # Mission logic
```

---

## License

This project is provided for educational purposes.

---

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Submit a pull request

---

## Acknowledgments

- [ROS 2 Navigation2](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [Gazebo](https://gazebosim.org/)
