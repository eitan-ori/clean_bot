# Clean Bot Description

URDF/Xacro model of the Clean Bot robot for ROS 2.

## Robot Structure

```
base_link
├── chassis (0.3m × 0.3m × 0.15m white box)
│   ├── laser (RPLidar A1M8, top-mounted)
│   ├── imu_link (Grove IMU 9DOF)
│   └── ultrasonic_link (HC-SR04, front-facing)
├── left_wheel (driven, 0.067m diameter)
├── right_wheel (driven, 0.067m diameter)
└── caster_wheel (passive front caster)
```

## Files

| File | Description |
|------|-------------|
| `urdf/robot.urdf.xacro` | Top-level robot assembly |
| `urdf/robot_core.xacro` | Chassis, wheels, caster |
| `urdf/lidar.xacro` | 2D LiDAR sensor mount |
| `urdf/imu.xacro` | IMU sensor mount |
| `urdf/ultrasonic.xacro` | Ultrasonic sensor mount |
| `urdf/inertial_macros.xacro` | Inertia calculation helpers |
| `launch/rsp.launch.py` | Robot State Publisher launch |

## Usage

```bash
# Launch robot state publisher only
ros2 launch clean_bot_description rsp.launch.py

# Visualize in RViz
ros2 launch clean_bot_description visualize_robot.launch.py
```

## Physical Parameters

| Property | Value |
|----------|-------|
| Chassis | 0.3m × 0.3m × 0.15m |
| Wheel Diameter | 0.067m |
| Wheel Separation | 0.20m |
| Total Mass | ~0.5 kg |
