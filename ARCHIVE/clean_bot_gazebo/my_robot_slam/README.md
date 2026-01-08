# My Robot SLAM

> ⚠️ **DEPRECATED**: This package is outdated. Please use the following packages instead:
> - **clean_bot_hardware**: Hardware drivers (Arduino, IMU, Lidar)
> - **clean_bot_mission**: Mission nodes (coverage, exploration)
> - **clean_bot_navigation**: Navigation configuration
> 
> The main launch file is now: `ros2 launch clean_bot_hardware robot_bringup.launch.py`

---

## Legacy Documentation

This package implements a ROS2 system for a robot that maps, vacuums, and autonomously docks. It includes drivers for Arduino, IMU, Lidar, and nodes for SLAM, Navigation, Coverage Planning, and Auto-Docking.

## Architecture

- **Arduino Bridge**: Communicates with the Arduino microcontroller for motor control and odometry.
- **IMU Driver**: Reads data from the ICM20600 IMU.
- **SLAM Toolbox**: Performs Simultaneous Localization and Mapping.
- **Nav2**: Handles path planning and navigation.
- **Coverage Planner**: Generates a path to cover the entire accessible area (vacuuming).
- **Auto Docker**: Autonomous docking system using Lidar-based V-shape detection.
  - `dock_detector.py`: Detects the charging dock using a V-shape pattern in LaserScan data.
  - `auto_docker.py`: Controls the robot to approach the detected dock using a P-controller.

## Prerequisites

- ROS2 Humble
- `slam_toolbox`
- `navigation2`
- `nav2_bringup`
- `robot_localization`
- `pyserial`
- `smbus2`

## Build

```bash
cd ~/robot_ws
colcon build --packages-select my_robot_slam
source install/setup.bash
```

## Run

### 1. Visual Simulation (Demo)
To see the auto-docking system in action with a visual interface (RViz):

```bash
ros2 launch my_robot_slam visual_demo.launch.py
```
This will launch:
- `simple_simulation`: A kinematic simulator for the robot.
- `dock_detector`: The perception node.
- `auto_docker`: The control node.
- `rviz2`: Visualization tool pre-configured to show the robot, laser scan, and dock pose.

### 2. Full System (Real Robot)
To launch the entire system on the physical robot:

```bash
ros2 launch my_robot_slam main.launch.py
```

### 3. Hardware Drivers Only
To launch just the hardware drivers and state publisher:

```bash
ros2 launch my_robot_slam bringup.launch.py
```

## Testing

This package enforces strict code style (flake8, pep257) and includes both unit and integration tests.

### Running Tests
To run the full test suite (including linting):

```bash
colcon test --packages-select my_robot_slam --event-handlers console_direct+
```

### Test Descriptions
- **`test_simulation_integration.py`**: Launches the full simulation stack and verifies that the robot successfully docks (reaches the target state) within 60 seconds.
- **`test_docking_logic.py`**: Unit tests for the `DockDetector` class logic.
- **Linting**: Checks for PEP8 compliance (`flake8`), docstring style (`pep257`), and CMake/XML validity.


```bash
colcon test --packages-select my_robot_slam
colcon test-result --all
```

## Hardware Setup

- **Arduino**: Flash `arduino_code/arduino_ros_node.ino` to the board. Connect via USB.
- **Lidar**: RPLidar connected via USB.
- **IMU**: ICM20600 connected via I2C.
