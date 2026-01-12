# Telegram Bridge for Clean Bot

Control your Clean Bot robot remotely via Telegram!

## Architecture

```
┌─────────────────────┐         Network          ┌─────────────────────┐
│     Your PC         │◄───────(ROS2 DDS)───────►│   Raspberry Pi      │
│                     │                          │                     │
│  telegram_bridge.py │                          │  full_mission       │
│  - Telegram Bot     │   /mission_command ────► │  - exploration      │
│  - ROS2 Node        │ ◄── /mission_state       │  - coverage         │
│                     │ ◄── /map                 │  - nav2, slam       │
└─────────────────────┘                          └─────────────────────┘
```

## Setup

### On your PC

1. **Install Python dependencies:**
   ```bash
   pip install python-telegram-bot pillow numpy
   ```

2. **Make sure ROS2 is installed and sourced:**
   ```bash
   source /opt/ros/humble/setup.bash
   ```

3. **Set the same ROS_DOMAIN_ID as the robot:**
   ```bash
   export ROS_DOMAIN_ID=0  # Must match the Pi
   ```

4. **Copy the script to your PC:**
   ```bash
   # Copy from robot workspace or download directly
   scp pi@<robot_ip>:~/robot_ws/src/clean_bot_mission/scripts/telegram_bridge.py .
   ```

5. **Edit the script to add your Telegram token:**
   ```python
   TELEGRAM_TOKEN = 'your_token_from_botfather'
   ALLOWED_USER_IDS = [your_telegram_id]  # Optional: restrict access
   ```

### On the Raspberry Pi (Robot)

1. **Launch the robot:**
   ```bash
   cd ~/robot_ws
   source install/setup.bash
   ros2 launch clean_bot_mission cleaning_mission.launch.py
   ```

## Running

1. **First, start the robot on the Pi** (see above)

2. **Then, run the Telegram bridge on your PC:**
   ```bash
   python3 telegram_bridge.py
   ```

3. **Open Telegram and send `/start` to your bot**

## Telegram Commands

| Command | Description |
|---------|-------------|
| `/start` | Show welcome message and available commands |
| `/scan` | Start exploration/scanning |
| `/stopscan` | Stop scanning and wait for clean command |
| `/clean` | Start cleaning/coverage |
| `/stopclean` | Stop cleaning |
| `/home` | Return to home position |
| `/reset` | Reset mission to initial state |
| `/pause` | Pause current operation |
| `/resume` | Resume from pause |
| `/status` | Get current mission state |
| `/map` | Get map image with robot position |
| `/help` | Show help |

## Typical Workflow

1. Start the robot on the Pi
2. Start the Telegram bridge on your PC
3. Send `/scan` to start exploration
4. Wait for exploration to complete, or send `/stopscan`
5. Send `/clean` to start cleaning
6. Send `/map` to see progress
7. Send `/stopclean` when done, or wait for completion

## Network Configuration

For ROS2 communication to work between PC and Pi:

1. **Both machines must be on the same network**

2. **Same ROS_DOMAIN_ID:**
   ```bash
   export ROS_DOMAIN_ID=0
   ```

3. **Firewall must allow ROS2 DDS traffic:**
   - UDP ports 7400-7500 (default DDS ports)
   - Or disable firewall temporarily for testing

4. **If using WiFi:** Make sure multicast is enabled on your router

### Troubleshooting Network Issues

**Check if topics are visible:**
```bash
# On PC
ros2 topic list
# Should show /mission_command, /mission_state, /map, etc.
```

**Test communication:**
```bash
# On PC - send a test command
ros2 topic pub --once /mission_command std_msgs/msg/String "data: 'status'"

# On Pi - check if received
ros2 topic echo /mission_command
```

**If topics not visible:**
```bash
# Try setting RMW implementation explicitly
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

## Creating a Telegram Bot

1. Open Telegram and search for `@BotFather`
2. Send `/newbot`
3. Follow the instructions to create your bot
4. Copy the token and paste it in `telegram_bridge.py`

## Security Notes

- **Keep your token secret!** Don't commit it to public repositories
- Use `ALLOWED_USER_IDS` to restrict who can control the robot
- To find your Telegram ID, send a message to `@userinfobot`

## Example Session

```
You: /start
Bot: 🤖 Clean Bot Controller
     Available commands:
     ...

You: /scan
Bot: 🔍 Starting scan...
     Robot will explore the environment.

You: /status
Bot: 📊 Robot Status
     Mission State: 🔍 EXPLORING
     ...

You: /map
Bot: 🗺️ [Map image with robot position]

You: /stopscan
Bot: 🛑 Stopping scan...

You: /clean
Bot: 🧹 Starting cleaning...

You: /map
Bot: 🗺️ [Map image showing coverage progress]

You: /stopclean
Bot: 🛑 Stopping cleaning...

You: /home
Bot: 🏠 Returning home...
```
