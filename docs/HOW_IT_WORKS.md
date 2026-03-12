# How Clean Bot Works: A Detailed System Flow

This document explains exactly what happens under the hood when you run the Clean Bot project, from the moment the hardware is powered on to the execution of an autonomous mission.

## 1. System Initialization (The Bringup)

When you run `ros2 launch clean_bot_hardware robot_bringup.launch.py`, the following sequence starts:

### A. Hardware Communication
- **Arduino Driver**: Opens the serial port (default: `/dev/ttyUSB0`). It reads ultrasonic range data and controls the motors, cleaning relay, and servo.
- **LiDAR Driver**: Starts the RPLidar A1M8 scanner (default: `/dev/ttyUSB1`). It spins up the motor and begins broadcasting 360° distance measurements on the `/scan` topic.
- **IMU Node**: Accesses the I2C bus to read the ICM20600 (motion) and AK09918 (magnetometer) sensors. It publishes raw acceleration and angular rates.
- **Scan Throttler**: Reduces the `/scan` topic rate from 10Hz to 5Hz, publishing `/scan_throttled` for use by rf2o laser odometry.

### B. Structural Definition
- **Robot State Publisher**: Reads the Xacro files (URDF) and compiles them into a description. It calculates the fixed transforms (e.g., where the LiDAR is relative to the chassis) and publishes the TF tree.

---

## 2. Perception & Sensor Processing

Raw data is noisy and insufficient on its own. The system processes it through two main stages:

1.  **Orientation Filtering (Madgwick)**:
    - The raw IMU and Magnetometer data are fused.
    - **Result**: A stable estimate of the robot's heading (Absolute orientation) is published to `/imu/data`.
2.  **Laser Odometry (rf2o)**:
    - The `rf2o_laser_odometry` node subscribes to `/scan_throttled` (5Hz laser scans).
    - It uses consecutive scan matching to estimate the robot's motion.
    - **Result**: It publishes the `/odom` topic and the transform between the `odom` frame and the `base_link` frame. This tells the system where the robot is relative to its starting point.

---

## 3. Mapping & SLAM

With a filtered odometry (`odom` -> `base_link`) and a LiDAR scan (`/scan`), the **SLAM Toolbox** takes over:

- **Scan Matching**: It compares incoming laser scans to previous ones to build a map.
- **Loop Closure**: It recognizes if the robot has returned to a previously seen area and corrects for drift.
- **Global Localization**: It calculates the transform between the `map` frame and the `odom` frame.
- **Result**: You see a live-updating map in RViz, and the robot knows exactly where it sits within that map.

---

## 4. Navigation & Obstacle Avoidance

When a mission is started, the system uses **Nav2**:

### A. Planning
- **Global Planner**: Uses the map to find the shortest path to the target waypoint.
- **Local Planner**: Looks at the real-time LiDAR scan and the **Ultrasonic Point Cloud** (converted from range data by `low_obstacle_detector.py`) to steer around objects in the immediate path.

### B. Velocity Output
- Nav2 calculates the required speeds and outputs a velocity command to `/cmd_vel_nav`.

---

## 5. Safety & Physical Motion

Before a command reaches the motors, it passes through the **Emergency Stop (E-Stop)** node:

1.  **The Filter**: The E-Stop node monitors the `/ultrasonic_range` topic.
2.  **The Intervention**:
    - If an obstacle is <30cm away, it scales down the velocity from `/cmd_vel_nav` (Caution).
    - If an obstacle is <10cm away, it forces the velocity to zero (Stop).
3.  **The Command**: The final vetted velocity is published to `/cmd_vel`.
4.  **The Hardware Action**: The Arduino Driver converts `/cmd_vel` into PWM values for the left and right motors.

---

## 6. Telegram Control Flow

The robot is controlled remotely via Telegram:

1.  **Telegram Bridge** (`telegram_bridge.py`): Runs on a PC and connects to both the Telegram Bot API and ROS2 topics.
2.  **Commands**: User sends commands (e.g., `start_scan`, `start_clean`) via Telegram chat. The bridge publishes these to `/mission_command`.
3.  **Mission Controller** (`full_mission.py`): Receives commands and transitions through states (EXPLORING → COVERAGE → RETURNING).
4.  **Status Feedback**: The mission controller publishes state updates to `/mission_state`, which the Telegram bridge relays back to the user.

---

## Summary of the Data Loop

| Source | Topic | Destination | Purpose |
| :--- | :--- | :--- | :--- |
| **LiDAR** | `/scan` | Nav2 / SLAM / Throttle | Obstacle detection & mapping |
| **Throttle** | `/scan_throttled` | rf2o | Reduced-rate scans for odometry |
| **IMU** | `/imu/data` | Nav2 | High-accuracy rotation data |
| **rf2o** | `/odom`, `odom -> base_link` | Nav2 | Where am I relative to start? |
| **SLAM** | `map -> odom` | System | Where am I in the world map? |
| **Nav2** | `/cmd_vel_nav` | E-Stop Node | Desired movement path |
| **E-Stop** | `/cmd_vel` | Arduino Driver | Safe movement command |
| **Telegram** | `/mission_command` | Mission Controller | User commands from Telegram |
| **Mission** | `/mission_state` | Telegram Bridge | Status updates to user |
