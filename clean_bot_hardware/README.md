# Clean Bot Hardware

ROS 2 hardware drivers and main bringup for the Clean Bot robot.

## 🤖 Hardware Components

| Component | Model | Connection | Driver |
|-----------|-------|------------|--------|
| **Motors** | GB37-131 DC Motors | Arduino (L298N) | `arduino_driver` |
| **Ultrasonic** | HC-SR04 | Arduino (GPIO) | `arduino_driver` |
| **Lidar** | RPLIDAR A1M8 Gen6 | USB Serial | `sllidar_ros2` |
| **IMU** | Grove 9DOF (ICM20600+AK09918) | I2C | `imu_publisher` |

## 📁 Package Structure

```
clean_bot_hardware/
├── clean_bot_hardware/          # Python modules
│   ├── arduino_driver.py        # ⭐ Motor/ultrasonic/cleaning driver
│   ├── emergency_stop.py        # Safety velocity filter
│   ├── low_obstacle_detector.py # Ultrasonic → PointCloud2
│   ├── imu_publisher_node.py    # IMU ROS publisher
│   ├── simple_imu_driver.py     # Low-level IMU I2C driver
│   ├── rplidar_test.py          # Lidar diagnostic tool
│   └── imu_odom_broadcaster.py  # TF broadcaster (legacy)
├── config/
│   ├── ekf.yaml                 # Robot Localization EKF config
│   ├── nav2_params.yaml         # Nav2 parameters
│   └── mapper_params_online_async.yaml  # SLAM Toolbox config
├── launch/
│   ├── robot_bringup.launch.py  # ⭐ MAIN LAUNCH FILE
│   ├── sensors.launch.py        # Sensors only (Lidar+IMU)
│   └── slam.launch.py           # SLAM + sensors
└── scripts/
    └── check_lidar.py           # Diagnostic script
```

## 🚀 Quick Start

### 1. Hardware Setup

1. **Arduino**: Flash the code from `my_robot_slam/arduino_code/arduino_ros_node.ino`
2. **Connections**:
   - Arduino → USB (e.g., `/dev/ttyUSB0`)
   - RPLIDAR → USB (e.g., `/dev/ttyUSB1`)
   - IMU → I2C (bus 1, default)

3. **Find your ports**:
   ```bash
   ls /dev/ttyUSB*
   # or
   ls /dev/ttyACM*
   ```

4. **Set permissions** (one-time):
   ```bash
   sudo usermod -a -G dialout $USER
   # Log out and back in
   ```

### 2. Build the Package

```bash
cd ~/robot_ws
colcon build --packages-select clean_bot_hardware clean_bot_mission
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

## ⚙️ Robot Calibration

### Physical Parameters (CRITICAL!)

Edit these in launch or pass as arguments:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `wheel_radius` | 0.034m | Wheel radius (measure your wheels!) |
| `wheel_separation` | 0.20m | Distance between wheels |

## 📡 Topics

### Published

| Topic | Type | Description |
|-------|------|-------------|
| `/odom` | nav_msgs/Odometry | Laser-based odometry (rf2o) |
| `/ultrasonic_range` | sensor_msgs/Range | Distance from ultrasonic |
| `/scan` | sensor_msgs/LaserScan | Lidar scan data |
| `/imu/data_raw` | sensor_msgs/Imu | Raw IMU (accel+gyro) |
| `/imu/mag` | sensor_msgs/MagneticField | Magnetometer |
| `/imu/data` | sensor_msgs/Imu | Filtered IMU (with orientation) |
| `/odom` | nav_msgs/Odometry | Fused odometry (from EKF) |
| `/map` | nav_msgs/OccupancyGrid | SLAM map |

### Subscribed

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | geometry_msgs/Twist | Velocity commands |

## 🗺️ Running a Cleaning Mission

After launching the robot:

1. **Manual exploration** (to create initial map):
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
   Drive around the room edges to let SLAM build the map.

2. **Start coverage mission** (14cm cleaning width):
   ```bash
   ros2 run clean_bot_mission coverage_mission --ros-args -p coverage_width:=0.14
   ```

3. **Save the map** (optional):
   ```bash
   ros2 run nav2_map_server map_saver_cli -f ~/my_map
   ```

## 🔧 Troubleshooting

### Arduino not connecting
```bash
# Check if device exists
ls -la /dev/ttyUSB*

# Check permissions
groups  # Should include 'dialout'

# Try with sudo (temporary fix)
sudo chmod 666 /dev/ttyUSB0
```

### IMU not detected
```bash
# Check I2C devices
i2cdetect -y 1
# Should show devices at 0x69 (IMU) and 0x0C (Magnetometer)
```

### TF errors in RViz
```bash
ros2 run tf2_tools view_frames
evince frames.pdf
```

## 📦 Dependencies

Install with:
```bash
sudo apt install ros-humble-slam-toolbox ros-humble-navigation2 \
    ros-humble-nav2-bringup ros-humble-robot-localization \
    ros-humble-imu-filter-madgwick ros-humble-teleop-twist-keyboard

pip3 install pyserial smbus2
```
