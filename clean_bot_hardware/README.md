# Clean Bot Hardware Package

This ROS 2 package contains hardware drivers and launch files for the Clean Bot robot sensors.

## Sensors Supported

### 1. SLAMTEC Lidar (RPLidar A1/A2/etc.)
- Uses the `sllidar_ros2` package
- Publishes: `/scan` (sensor_msgs/LaserScan)
- Default port: `/dev/ttyUSB0`

### 2. Grove IMU 9DOF (ICM20600 + AK09918)
- Custom Python driver using smbus2
- Publishes:
  - `/imu/data_raw` (sensor_msgs/Imu) - Raw accelerometer and gyroscope data
  - `/imu/mag` (sensor_msgs/MagneticField) - Magnetometer data
  - `/imu/data` (sensor_msgs/Imu) - Filtered orientation (when using Madgwick filter)

## Installation on Raspberry Pi

### Prerequisites

```bash
# Install smbus2 for I2C communication
pip3 install smbus2

# Install ROS 2 Madgwick filter
sudo apt install ros-humble-imu-filter-madgwick
```

### Clone and Build

```bash
cd ~/robot_ws/src
git clone <your-repo-url> clean_bot_hardware

cd ~/robot_ws
colcon build --packages-select clean_bot_hardware sllidar_ros2
source install/setup.bash
```

## Launch Files

### Launch All Sensors
```bash
ros2 launch clean_bot_hardware sensors.launch.py
```

### Launch IMU Only
```bash
ros2 launch clean_bot_hardware imu.launch.py
```

### Launch with Custom Parameters
```bash
ros2 launch clean_bot_hardware sensors.launch.py \
    serial_port:=/dev/ttyUSB0 \
    serial_baudrate:=115200 \
    i2c_bus:=1 \
    use_madgwick:=true \
    use_mag:=true
```

## Parameters

### Lidar Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `serial_port` | `/dev/ttyUSB0` | Serial port for lidar |
| `serial_baudrate` | `115200` | Baudrate |
| `lidar_frame_id` | `laser` | TF frame ID |

### IMU Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `i2c_bus` | `1` | I2C bus number |
| `imu_frame_id` | `imu_link` | TF frame ID |
| `imu_rate` | `50.0` | Publish rate (Hz) |
| `use_madgwick` | `true` | Enable Madgwick filter |
| `use_mag` | `true` | Use magnetometer for yaw |

## Troubleshooting

### IMU not detected
1. Check I2C is enabled: `sudo raspi-config` → Interface Options → I2C
2. Verify connection: `i2cdetect -y 1` (should show 0x69 and 0x0C)
3. Check permissions: `sudo usermod -a -G i2c $USER`

### Lidar not detected
1. Check USB connection: `ls /dev/ttyUSB*`
2. Add udev rules for persistent naming
3. Check permissions: `sudo usermod -a -G dialout $USER`
