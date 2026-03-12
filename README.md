# Clean Bot — Autonomous Cleaning Robot

A physical differential-drive cleaning robot built with ROS 2 Humble, controlled via Telegram. The robot autonomously maps a room using SLAM, performs adaptive coverage cleaning, and returns home — all triggered by simple chat commands.

![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue)
![Nav2](https://img.shields.io/badge/Nav2-Navigation-green)
![SLAM](https://img.shields.io/badge/SLAM-Toolbox-purple)
![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi-red)

---

## Architecture

```
  Telegram (PC)                    Raspberry Pi
 ┌──────────────┐        ┌─────────────────────────────────────────────┐
 │  User sends  │  ROS 2 │                                             │
 │  /scan or    │◄──────►│  full_mission_controller                    │
 │  /clean      │ topics │       │                                     │
 │              │        │       ▼                                     │
 │ telegram_    │        │  ┌──────────┐   ┌──────────┐               │
 │ bridge.py    │        │  │ Frontier │   │ Coverage  │               │
 │              │        │  │ Explorer │   │ Planner   │               │
 └──────────────┘        │  └────┬─────┘   └────┬──────┘               │
                         │       │               │                     │
                         │       ▼               ▼                     │
                         │  ┌──────────┐   ┌──────────┐               │
                         │  │  SLAM    │   │   Nav2   │               │
                         │  │ Toolbox  │   │  Stack   │               │
                         │  └────┬─────┘   └────┬──────┘               │
                         │       │               │                     │
                         │       ▼               ▼                     │
                         │    /map, /tf       /cmd_vel                 │
                         │                       │                     │
                         │  ┌────────────────────┼──────────────────┐  │
                         │  │          Hardware Layer                │  │
                         │  │                    │                   │  │
                         │  │  RPLidar ──► /scan │                  │  │
                         │  │  rf2o ────► /odom  ▼                  │  │
                         │  │  IMU ────► /imu   Arduino ──► Motors  │  │
                         │  │                   (L298N + DC motors)  │  │
                         │  └───────────────────────────────────────┘  │
                         └─────────────────────────────────────────────┘
```

### TF Tree

```
map → odom                       (SLAM Toolbox)
       → base_link               (rf2o laser odometry)
            ├── laser             (RPLidar A1M8)
            ├── imu_link          (Grove IMU 9DOF)
            ├── ultrasonic_link   (HC-SR04)
            ├── left_wheel
            ├── right_wheel
            └── caster_wheel
```

---

## Hardware

| Component | Model | Role |
|-----------|-------|------|
| SBC | Raspberry Pi 4 | ROS 2 host |
| Microcontroller | Arduino | Motor control, sensors, cleaning actuators |
| Motor Driver | L298N | Drives two DC motors |
| Motors | GB37-131 DC | Differential drive (wheel ⌀ ~0.068 m, separation ~0.20 m) |
| LiDAR | RPLidar A1M8 | 2D 360° scanning |
| IMU | Grove IMU 9DOF (ICM20600 + AK09918) | Orientation |
| Range Sensor | HC-SR04 Ultrasonic | Close obstacle detection |
| Cleaning | Relay + Servo (via Arduino) | Cleaning mechanism control |

---

## Prerequisites

- **On the Raspberry Pi:**
  - Ubuntu 22.04 + ROS 2 Humble (desktop or base)
  - System packages:
    ```bash
    sudo apt install -y \
        ros-humble-navigation2 ros-humble-nav2-bringup \
        ros-humble-slam-toolbox ros-humble-xacro \
        ros-humble-robot-state-publisher ros-humble-joint-state-publisher
    ```
  - Python packages: `python-telegram-bot`, `opencv-python`, `numpy`

- **On your PC (Telegram bridge only):**
  - ROS 2 Humble sourced
  - Same `ROS_DOMAIN_ID` as the Pi
  - Python packages: `python-telegram-bot`

### Build

```bash
cd ~/robot_ws
colcon build
source install/setup.bash
```

> **Note:** `rf2o_laser_odometry` is a git submodule. After cloning, run:
> ```bash
> git submodule update --init --recursive
> ```

---

## Quick Start

### 1. On the Raspberry Pi — Launch the robot

```bash
source ~/robot_ws/install/setup.bash
ros2 launch clean_bot_mission cleaning_mission.launch.py
```

This single command brings up the entire stack:
- Robot state publisher (URDF)
- RPLidar driver + rf2o laser odometry
- SLAM Toolbox (online async)
- Nav2 navigation stack
- Full mission controller (exploration + coverage + return home)

### 2. On your PC — Start the Telegram bridge

```bash
export ROS_DOMAIN_ID=<same as Pi>
export CLEANBOT_TELEGRAM_TOKEN='your_token_from_botfather'
source ~/robot_ws/install/setup.bash
ros2 run clean_bot_mission telegram_bridge
```

### 3. In Telegram — Control the robot

Send commands to your bot. The robot handles the rest.

---

## Telegram Commands

| Command | Description |
|---------|-------------|
| `/start` | Show welcome message and available commands |
| `/scan` | Explore the room — frontier-based exploration builds a SLAM map |
| `/stopscan` | Stop exploration and wait for cleaning command |
| `/clean` | Clean the room — adaptive coverage path over the mapped area |
| `/stopclean` | Stop cleaning |
| `/home` | Return to starting position |
| `/reset` | Reset mission to initial state |
| `/pause` | Pause current operation |
| `/resume` | Resume paused operation |
| `/status` | Get the current mission state and sensor info |
| `/map` | Get the current map as an image with the robot's position marked |
| `/help` | Show help |

After cleaning completes (or the robot returns home), the bot automatically sends the final map to all users who have interacted with it.

---

## Key ROS 2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/mission_command` | `std_msgs/String` | Commands from Telegram bridge → mission controller |
| `/mission_state` | `std_msgs/String` | Current state published by mission controller |
| `/map` | `nav_msgs/OccupancyGrid` | SLAM Toolbox map |
| `/cmd_vel_nav` | `geometry_msgs/Twist` | Nav2 velocity commands → emergency stop filter |
| `/cmd_vel` | `geometry_msgs/Twist` | Filtered velocity commands → Arduino |
| `/scan` | `sensor_msgs/LaserScan` | RPLidar laser scans (10 Hz) |
| `/scan_throttled` | `sensor_msgs/LaserScan` | Throttled scans for rf2o (5 Hz) |
| `/odom` | `nav_msgs/Odometry` | rf2o laser-based odometry |

---

## Packages

| Package | Description |
|---------|-------------|
| **clean_bot_description** | Robot URDF/Xacro model — chassis, wheels, sensor frames |
| **clean_bot_hardware** | Hardware drivers, Nav2/SLAM configs, launch files for bringup |
| **clean_bot_mission** | Mission control — frontier explorer, coverage planner, full mission controller, Telegram bridge |
| **sllidar_ros2** | RPLidar A1M8 driver (external package) |
| **rf2o_laser_odometry** | Laser scan matching odometry (external, git submodule) |

---

## Troubleshooting

### No `odom → base_link` transform

rf2o laser odometry needs valid `/scan` data. Check the lidar:
```bash
ros2 topic hz /scan            # Should be ~5-10 Hz
ros2 run tf2_ros tf2_echo odom base_link
```
If no data, verify the RPLidar USB connection (`/dev/ttyUSB0`) and permissions (`sudo chmod 666 /dev/ttyUSB0`).

### Robot doesn't move

Check that Arduino is connected and `/cmd_vel` is being published:
```bash
ros2 topic echo /cmd_vel
```
Verify the serial port in the hardware launch config.

### Nav2 nodes not activating

SLAM must publish the `/map` topic before Nav2 starts. Wait for the lidar to produce scans and rf2o to publish odometry:
```bash
ros2 node list | grep -E "bt_navigator|controller_server|planner_server"
```

### Telegram bot not receiving commands

Ensure `ROS_DOMAIN_ID` matches on both Pi and PC. Verify network connectivity:
```bash
ros2 topic list    # Run on PC — should show Pi topics
```

### Useful debug commands

```bash
ros2 node list                          # All active nodes
ros2 run tf2_tools view_frames          # Save TF tree as PDF
ros2 topic hz /scan /odom               # Check sensor rates
ros2 topic echo /mission_state          # Monitor mission progress
```

---

## File Structure

```
├── clean_bot_description/       # Robot URDF model
│   ├── launch/
│   │   └── rsp.launch.py       # Robot State Publisher
│   └── urdf/
│       ├── robot.urdf.xacro    # Main entry point
│       ├── robot_core.xacro    # Chassis, wheels, caster
│       ├── lidar.xacro         # RPLidar frame
│       ├── imu.xacro           # IMU frame
│       ├── ultrasonic.xacro    # Ultrasonic sensor frame
│       └── inertial_macros.xacro
│
├── clean_bot_hardware/          # Hardware drivers & configs
│   ├── config/                  # Nav2 params, SLAM params
│   └── launch/
│       └── robot_bringup.launch.py
│
├── clean_bot_mission/           # Mission control & Telegram
│   ├── clean_bot_mission/
│   │   ├── frontier_explorer.py     # Autonomous frontier-based exploration
│   │   ├── adaptive_coverage.py     # Map-based coverage planner
│   │   ├── simple_coverage.py       # Fallback boustrophedon coverage
│   │   └── full_mission.py          # Mission state machine (scan→clean→home)
│   ├── launch/
│   │   └── cleaning_mission.launch.py
│   └── scripts/
│       └── telegram_bridge.py
│
├── rf2o_laser_odometry/         # Laser odometry (git submodule)
├── sllidar_ros2/                # RPLidar driver
├── docs/                        # Additional documentation
└── README.md
```

---

## Acknowledgments

- [ROS 2 Navigation2](https://navigation.ros.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [rf2o_laser_odometry](https://github.com/MAPIRlab/rf2o_laser_odometry)
- [Slamtec RPLidar](https://github.com/Slamtec/sllidar_ros2)
