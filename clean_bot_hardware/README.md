# Clean Bot Hardware

ROS 2 hardware drivers and main bringup for the Clean Bot robot.

## Hardware Components

| Component | Model | Connection | Driver |
|-----------|-------|------------|--------|
| **Motors** | GB37-131 DC Motors | Arduino (L298N) | `arduino_driver` |
| **Ultrasonic** | HC-SR04 | Arduino (GPIO) | `arduino_driver` |
| **Lidar** | RPLIDAR A1M8 Gen6 | USB Serial | `sllidar_ros2` |
| **IMU** | Grove 9DOF (ICM20600+AK09918) | I2C | `imu_publisher` |

## Package Structure

```
clean_bot_hardware/
├── clean_bot_hardware/          # Python modules
│   ├── arduino_driver.py        # Motor/ultrasonic/cleaning driver
│   ├── emergency_stop.py        # Safety velocity filter (cmd_vel_nav → cmd_vel)
│   ├── low_obstacle_detector.py # Ultrasonic → PointCloud2
│   ├── imu_publisher_node.py    # IMU ROS publisher
│   ├── simple_imu_driver.py     # Low-level IMU I2C driver
│   ├── scan_throttler.py        # Throttles /scan (10Hz → 5Hz) for rf2o
│   └── rplidar_test.py          # Lidar diagnostic tool
├── config/
│   ├── nav2_params.yaml         # Nav2 parameters
│   ├── mapper_params_online_async.yaml  # SLAM Toolbox config
│   ├── rplidar_a1.yaml          # RPLidar parameters
│   └── rplidar_rviz.rviz        # RViz config for lidar viewing
├── launch/
│   ├── robot_bringup.launch.py  # MAIN LAUNCH FILE
│   ├── sensors.launch.py        # Sensors only (Lidar+IMU)
│   ├── slam.launch.py           # SLAM + sensors
│   └── rviz_lidar.launch.py     # Lidar visualization
└── scripts/
    └── check_lidar.py           # Diagnostic script
```

## Quick Start

### 1. Hardware Setup

1. **Arduino**: Flash the motor control firmware
2. **Connections**:
   - Arduino → USB (e.g., `/dev/ttyUSB0`)
   - RPLIDAR → USB (e.g., `/dev/ttyUSB1`)
   - IMU → I2C (bus 1, default)

3. **Find your ports**:
   ```bash
   ls /dev/ttyUSB* /dev/ttyACM*
   ```

4. **Set permissions** (one-time):
   ```bash
   sudo usermod -a -G dialout $USER
   # Log out and back in
   ```

### 2. Build the Package

```bash
cd ~/robot_ws
colcon build --packages-select clean_bot_hardware
source install/setup.bash
```

### 3. Launch the Robot

**Full system (mapping + navigation):**
```bash
ros2 launch clean_bot_hardware robot_bringup.launch.py
```

**With custom port assignments:**
```bash
ros2 launch clean_bot_hardware robot_bringup.launch.py \
    arduino_port:=/dev/ttyUSB0 \
    lidar_port:=/dev/ttyUSB1
```

**Sensors only (for testing):**
```bash
ros2 launch clean_bot_hardware sensors.launch.py
```

## Robot Calibration

### Physical Parameters

Edit these in launch or pass as arguments:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `wheel_radius` | 0.034m | Wheel radius |
| `wheel_separation` | 0.20m | Distance between wheels |

## Topics

### Published

| Topic | Type | Description |
|-------|------|-------------|
| `/odom` | nav_msgs/Odometry | Laser-based odometry (rf2o) |
| `/ultrasonic_range` | sensor_msgs/Range | Distance from ultrasonic |
| `/scan` | sensor_msgs/LaserScan | Lidar scan data (10 Hz) |
| `/scan_throttled` | sensor_msgs/LaserScan | Throttled scans for rf2o (5 Hz) |
| `/imu/data_raw` | sensor_msgs/Imu | Raw IMU (accel+gyro) |
| `/imu/mag` | sensor_msgs/MagneticField | Magnetometer |
| `/map` | nav_msgs/OccupancyGrid | SLAM map |

### Subscribed

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel_nav` | geometry_msgs/Twist | Velocity from Nav2 (→ emergency stop filter) |
| `/cmd_vel` | geometry_msgs/Twist | Filtered velocity → Arduino motor control |

## Velocity Safety Chain

```
Nav2 → /cmd_vel_nav → emergency_stop → /cmd_vel → arduino_driver → motors
```

The `emergency_stop` node filters velocity commands based on ultrasonic readings, stopping the robot if obstacles are detected within the safety threshold.

## Troubleshooting

### Arduino not connecting
```bash
ls -la /dev/ttyUSB*
groups  # Should include 'dialout'
sudo chmod 666 /dev/ttyUSB0  # Temporary fix
```

### IMU not detected
```bash
i2cdetect -y 1
# Should show devices at 0x69 (IMU) and 0x0C (Magnetometer)
```

### TF errors in RViz
```bash
ros2 run tf2_tools view_frames
evince frames.pdf
```

## Dependencies

```bash
sudo apt install ros-humble-slam-toolbox ros-humble-navigation2 \
    ros-humble-nav2-bringup ros-humble-teleop-twist-keyboard

pip3 install pyserial smbus2
```
