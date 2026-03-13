# Clean Bot — Complete Usage Instructions

A step-by-step guide to setting up and operating the Clean Bot autonomous cleaning robot.

---

## Table of Contents

1. [What You Need](#what-you-need)
2. [Physical Setup](#physical-setup)
3. [Software Installation](#software-installation)
4. [Building the Workspace](#building-the-workspace)
5. [Running the Robot (Raspberry Pi)](#running-the-robot-raspberry-pi)
6. [Running the Web Control Panel (PC)](#running-the-web-control-panel-pc)
7. [Using the Web Control Panel](#using-the-web-control-panel)
8. [Typical Cleaning Workflow](#typical-cleaning-workflow)
9. [Manual Driving](#manual-driving)
10. [Room Manager](#room-manager)
11. [Scheduling](#scheduling)
12. [Settings & Preferences](#settings--preferences)
13. [Telegram Bot (Alternative Control)](#telegram-bot-alternative-control)
14. [Troubleshooting](#troubleshooting)
15. [Shutting Down](#shutting-down)

---

## What You Need

### Hardware

| Component | Model | Port |
|-----------|-------|------|
| Single-board computer | Raspberry Pi 4 | — |
| Microcontroller | Arduino (Nano/Uno) | `/dev/ttyACM0` (USB) |
| LiDAR | RPLidar A1M8 | `/dev/ttyUSB0` (USB) |
| Motor driver | L298N | Connected to Arduino |
| Drive motors | GB37-131 DC × 2 | Connected to L298N |
| IMU | Grove IMU 9DOF (ICM20600 + AK09918) | I2C bus 1 |
| Range sensor | HC-SR04 Ultrasonic | Connected to Arduino |
| Cleaning system | Relay + Servo | Connected to Arduino |
| Power | Battery pack for motors + USB power bank for Pi | — |

### Software (Raspberry Pi)

- Ubuntu 22.04
- ROS 2 Humble (desktop or base install)
- Navigation2, SLAM Toolbox, and supporting ROS 2 packages

### Software (Control PC — any computer on the same network)

- ROS 2 Humble (sourced)
- Python 3.10+
- Flask, Flask-SocketIO, Pillow, NumPy, SciPy

---

## Physical Setup

1. **Mount all hardware** on the robot chassis:
   - Two drive wheels + one caster wheel (differential drive)
   - RPLidar A1M8 on top, facing outward (mounted 180° inverted)
   - Ultrasonic sensor at the front, low to the ground (3 cm height)
   - Arduino connected to the motor driver, ultrasonic, relay, and servo
   - IMU mounted near the center of mass

2. **Connect USB cables:**
   - Arduino → Pi USB (will appear as `/dev/ttyACM0`)
   - RPLidar → Pi USB (will appear as `/dev/ttyUSB0`)

3. **Set USB permissions** (do this once, or add a udev rule):
   ```bash
   sudo chmod 666 /dev/ttyACM0
   sudo chmod 666 /dev/ttyUSB0
   ```

4. **Power on** the robot (motor battery + Pi power).

---

## Software Installation

### On the Raspberry Pi

```bash
# Install ROS 2 packages
sudo apt install -y \
    ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-slam-toolbox ros-humble-xacro \
    ros-humble-robot-state-publisher ros-humble-joint-state-publisher \
    ros-humble-imu-filter-madgwick ros-humble-topic-tools

# Install Python dependencies
pip3 install pyserial smbus2 gpiozero numpy scipy
```

### On the Control PC

```bash
# Ensure ROS 2 Humble is installed and sourced
source /opt/ros/humble/setup.bash

# Install Python dependencies
pip3 install flask flask-socketio pillow numpy scipy
```

### Clone & Build (Both Machines)

```bash
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src
git clone <repository-url> .
git submodule update --init --recursive   # For rf2o_laser_odometry

cd ~/robot_ws
colcon build
source install/setup.bash
```

> **Tip:** Add `source ~/robot_ws/install/setup.bash` to your `~/.bashrc` so it's always available.

---

## Building the Workspace

After any code change:

```bash
cd ~/robot_ws
colcon build
source install/setup.bash
```

To build a specific package only:

```bash
colcon build --packages-select clean_bot_mission
```

---

## Running the Robot (Raspberry Pi)

### 1. Set ROS Domain ID (both Pi and PC must match)

```bash
export ROS_DOMAIN_ID=42    # Use any number 0-232, must match on both machines
```

### 2. Launch the robot stack

```bash
source ~/robot_ws/install/setup.bash
ros2 launch clean_bot_mission cleaning_mission.launch.py
```

This single command starts **everything** on the Pi:
- **Robot State Publisher** — broadcasts the URDF (robot model) and TF transforms
- **RPLidar A1 driver** — publishes `/scan` at 10 Hz
- **Scan throttler** — produces `/scan_throttled` at 5 Hz for odometry
- **rf2o laser odometry** — publishes `/odom` and the `odom → base_link` TF
- **IMU publisher** — publishes `/imu/data_raw` at 50 Hz
- **IMU Madgwick filter** — fuses IMU data, publishes `/imu/data`
- **Low obstacle detector** — converts ultrasonic to point cloud for Nav2 costmap
- **Emergency stop controller** — safety filter between Nav2 and motors
- **Arduino driver** — bridges ROS 2 commands to serial (motors + cleaning hardware)
- **SLAM Toolbox** — real-time mapping (publishes `/map`)
- **Nav2 navigation stack** — path planning, obstacle avoidance
- **Full mission controller** — state machine (scan → clean → go home)

#### Optional launch arguments

```bash
# Custom serial ports
ros2 launch clean_bot_mission cleaning_mission.launch.py \
    arduino_port:=/dev/ttyACM1 \
    lidar_port:=/dev/ttyUSB1

# Custom coverage width (cleaning path spacing)
ros2 launch clean_bot_mission cleaning_mission.launch.py coverage_width:=0.20

# Skip exploration and use an existing map
ros2 launch clean_bot_mission cleaning_mission.launch.py skip_exploration:=true
```

### 3. Verify the robot is running

```bash
# Check that all nodes are up
ros2 node list

# Check LiDAR is producing data
ros2 topic hz /scan

# Check odometry is working
ros2 topic echo /odom --once

# Check the TF tree
ros2 run tf2_tools view_frames   # Saves a PDF
```

---

## Running the Web Control Panel (PC)

### 1. Match the ROS Domain ID

```bash
export ROS_DOMAIN_ID=42    # Same as the Pi
```

### 2. Launch the web app

```bash
source ~/robot_ws/install/setup.bash
ros2 run clean_bot_mission web_control
```

You will see output like:

```
============================================================
🤖  Clean Bot Web Control Panel
============================================================

Network addresses (use any of these from another PC):
  → http://192.168.1.50:5000

Starting ROS 2 bridge...
ROS_DOMAIN_ID = 42
```

### 3. Open in your browser

Go to the URL printed in the terminal (e.g., `http://192.168.1.50:5000`) from **any device** on the same network — PC, tablet, or phone.

#### Optional: Custom port

```bash
ros2 run clean_bot_mission web_control -- --port 8080
```

---

## Using the Web Control Panel

### Layout

The web interface has two main sections:

| Area | What's there |
|------|-------------|
| **Left — Live Map** | Real-time SLAM map with robot position, obstacles, path trail, heatmap overlay |
| **Right — Controls** | Emergency stop, mission buttons, manual drive pad, room manager, stats, schedule, diagnostics, log |

### Header Bar

| Element | Purpose |
|---------|---------|
| 🔋 Battery | Estimated battery level (see [Battery Explanation](./BATTERY_EXPLANATION.md)) |
| Latency dot + ms | WebSocket round-trip latency (green < 100 ms, yellow < 500 ms, red > 500 ms) |
| 🔔 / 🔕 | Toggle notification chime sounds |
| EN / HE | Switch between English and Hebrew |
| ⚙️ | Open settings panel |
| 🌙 / ☀️ | Toggle dark / light theme |
| Green/red dot | WebSocket connection status |

### Mission Control Buttons

| Button | What it does |
|--------|-------------|
| 🔍 **Scan Room** | Start frontier-based exploration — the robot autonomously visits all unknowns to build a map |
| ⏹ **Stop Scan** | Stop exploration early (the partial map is kept) |
| 🧹 **Clean Room** | Start adaptive coverage cleaning — the robot cleans every reachable area of the map |
| ⏹ **Stop Clean** | Stop cleaning |
| 🏠 **Go Home** | Navigate back to the robot's starting position |
| 🔄 **Reset** | Reset the mission state machine to initial state |
| ⏸ **Pause** | Pause the current operation |
| ▶ **Resume** | Resume a paused operation |
| ⚠️ **EMERGENCY STOP** | **Immediately stop all robot movement** (big red button) |

> Buttons are automatically enabled/disabled based on the current mission state (e.g., you can't press "Clean" while scanning).

### Live Map Features

- **Zoom:** Scroll wheel or ＋/− buttons
- **Pan:** Click and drag the map
- **Click to navigate:** Click any point on the map → "Navigate Here" button appears
- **Heatmap overlay:** Toggle to see frequently-observed obstacle areas
- **Path trail:** Toggle to see where the robot has been
- **Export:** Download the current map as a PNG image

### Map Legend

| Color | Meaning |
|-------|---------|
| White | Free space (known, no obstacles) |
| Black | Wall or obstacle |
| Gray | Unknown (not yet explored) |
| Blue circle + arrow | Robot position and heading |
| Green line | Robot's path trail |

---

## Typical Cleaning Workflow

This is the recommended order of operations:

### Step 1 — Scan the room

1. Place the robot in the room you want to clean.
2. Click **🔍 Scan Room**.
3. Watch the map build up in real-time as the robot explores.
4. The robot automatically finishes when all frontiers are explored — or click **⏹ Stop Scan** to end early.
5. Optionally click **Save Room** in the Room Manager to save this map for later.

### Step 2 — Clean the room

1. Click **🧹 Clean Room**.
2. The robot follows an adaptive coverage path across the entire mapped area.
3. The cleaning hardware (relay/servo) activates automatically.
4. Watch the coverage progress bar in the status section.

### Step 3 — Go home

1. After cleaning finishes, the robot automatically returns to its starting position.
2. You can also click **🏠 Go Home** at any time to send it back.

### Step 4 — View the map

The final map is always visible in the Live Map section. Click **📥 Export Map** to save it as a PNG.

---

## Manual Driving

Use the D-pad in the **Manual Drive** section to control the robot directly:

| Control | Action |
|---------|--------|
| ↑ | Drive forward |
| ↓ | Drive backward |
| ← | Turn left |
| → | Turn right |
| STOP | Stop all movement |
| Speed slider | Adjust drive speed (m/s) |

### Keyboard Shortcuts

| Key | Action |
|-----|--------|
| `W` or `↑` | Forward |
| `S` or `↓` | Backward |
| `A` or `←` | Turn left |
| `D` or `→` | Turn right |
| `Space` | Stop |

> Manual drive sends velocity commands directly to the motors (through the emergency stop filter for safety).

---

## Room Manager

Save, load, and manage maps of rooms you've scanned.

| Action | How |
|--------|-----|
| **Save a room** | After scanning, type a name in the text box and click **Save Room** |
| **Preview a room** | Click a room name in the list — a popup shows the saved map |
| **Rename a room** | Click the pencil (✏️) icon next to a room name |
| **Delete a room** | Click the trash (🗑️) icon next to a room name |

Rooms are saved as JSON files on the server in the `saved_rooms/` directory.

---

## Scheduling

Set automatic cleaning schedules:

1. Expand the **Schedule** section.
2. Pick a time using the time picker.
3. Select the days of the week (Mon–Sun).
4. Click **+ Add**.
5. The robot will automatically start a scan+clean mission at the scheduled times.

To remove a schedule, click the ✕ button next to it.

---

## Settings & Preferences

Click the ⚙️ button in the header to open settings:

| Setting | What it controls |
|---------|-----------------|
| **Map Update Rate** | How often the map refreshes (0.2s – 5s) |
| **Default Drive Speed** | Default manual drive speed in m/s |
| **Notification Sound** | Enable/disable chime sounds |
| **Theme** | Dark or Light mode |
| **Language** | English or Hebrew |

Settings are saved in your browser's local storage — they persist across sessions.

---

## Telegram Bot (Alternative Control)

The robot can also be controlled via a Telegram bot (runs alongside or instead of the web panel).

### Start the Telegram bridge

```bash
export ROS_DOMAIN_ID=42
source ~/robot_ws/install/setup.bash
ros2 run clean_bot_mission telegram_bridge
```

### Available Commands

| Command | Description |
|---------|-------------|
| `/start` | Welcome message |
| `/scan` | Start room exploration |
| `/stopscan` | Stop scanning |
| `/clean` | Start cleaning |
| `/stopclean` | Stop cleaning |
| `/home` | Return to starting position |
| `/reset` | Reset mission state |
| `/pause` | Pause current operation |
| `/resume` | Resume operation |
| `/status` | Current state and sensor info |
| `/map` | Get the current map as an image |
| `/help` | Show help |

The bot automatically sends the final map when cleaning completes.

---

## Troubleshooting

### Robot doesn't move

1. **Check Arduino connection:**
   ```bash
   ls /dev/ttyACM*           # Should show /dev/ttyACM0
   sudo chmod 666 /dev/ttyACM0
   ```
2. **Check velocity commands reach the Arduino:**
   ```bash
   ros2 topic echo /cmd_vel
   ```
3. **Check the motor driver** — LEDs on the L298N should light up.

### No map appears / LiDAR not working

1. **Check LiDAR connection:**
   ```bash
   ls /dev/ttyUSB*           # Should show /dev/ttyUSB0
   sudo chmod 666 /dev/ttyUSB0
   ```
2. **Check scan data:**
   ```bash
   ros2 topic hz /scan       # Should be ~5-10 Hz
   ```
3. If no data, power-cycle the LiDAR (unplug and replug USB).

### No odometry / TF errors

The robot uses **laser odometry** (rf2o), not wheel encoders. It needs valid LiDAR data:
```bash
ros2 run tf2_ros tf2_echo odom base_link   # Should show transform
```

### Web panel shows "Disconnected"

1. **Ensure ROS_DOMAIN_ID matches** on both Pi and PC:
   ```bash
   echo $ROS_DOMAIN_ID      # Must be the same number on both machines
   ```
2. **Verify network connectivity:**
   ```bash
   ping <pi-ip-address>     # From PC
   ros2 topic list           # From PC — should show Pi topics
   ```
3. **Check firewall** — ROS 2 DDS uses multicast. Ensure UDP ports 7400–7500 are open.

### Nav2 not activating

Nav2 requires a published `/map` and valid TF tree. Wait for SLAM to produce a map:
```bash
ros2 topic echo /map --once  # Should print map data
ros2 node list | grep nav2   # Should show nav2 nodes
```

### Emergency stop keeps triggering

The ultrasonic sensor detects obstacles at < 10 cm. Check:
- Is something very close to the front of the robot?
- Is the ultrasonic sensor reading correctly?
  ```bash
  ros2 topic echo /ultrasonic_range
  ```

### Useful Debug Commands

```bash
ros2 node list                          # All active nodes
ros2 topic list                         # All active topics
ros2 topic hz /scan /odom /map          # Sensor frequencies
ros2 topic echo /mission_state          # Mission state changes
ros2 run tf2_tools view_frames          # Save TF tree as PDF
ros2 run rqt_graph rqt_graph            # Visualize node graph (if GUI available)
```

---

## Shutting Down

### Stop the web panel

Press `Ctrl+C` in the terminal running `web_control`.

### Stop the robot

Press `Ctrl+C` in the terminal running the launch file.

### Safe Power Off

```bash
# On the Pi
sudo shutdown -h now
```

Then disconnect motor battery power.

---

## Key Ports and Addresses Summary

| Resource | Default Value |
|----------|--------------|
| Arduino serial port | `/dev/ttyACM0` @ 57600 baud |
| LiDAR serial port | `/dev/ttyUSB0` @ 115200 baud |
| IMU I2C bus | Bus 1 |
| Web app URL | `http://<pc-ip>:5000` |
| ROS_DOMAIN_ID | `0` (change to match your setup) |
| Wheel radius | 0.0335 m |
| Wheel separation | 0.20 m |
| Coverage width | 0.14 m (adjustable via launch arg) |

---

## Quick Reference Card

```
┌────────────────────────────────────────────────────┐
│              CLEAN BOT — QUICK START               │
├────────────────────────────────────────────────────┤
│                                                    │
│  ON THE PI:                                        │
│    export ROS_DOMAIN_ID=42                         │
│    ros2 launch clean_bot_mission                   │
│         cleaning_mission.launch.py                 │
│                                                    │
│  ON YOUR PC:                                       │
│    export ROS_DOMAIN_ID=42                         │
│    ros2 run clean_bot_mission web_control           │
│                                                    │
│  IN YOUR BROWSER:                                  │
│    http://<pc-ip>:5000                             │
│                                                    │
│  WORKFLOW:                                         │
│    Scan Room → Clean Room → Go Home                │
│                                                    │
│  EMERGENCY:                                        │
│    Big red button or Space key = STOP              │
│                                                    │
└────────────────────────────────────────────────────┘
```
