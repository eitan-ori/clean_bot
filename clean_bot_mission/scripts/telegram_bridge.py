#!/usr/bin/env python3
"""
Telegram Bridge for Clean Bot

This script runs on your PC (not on the Pi) and communicates with the robot
via ROS2 topics over the network.

Architecture:
- PC: Runs this script (Telegram bot + ROS2 node)
- Pi: Runs the robot nodes (full_mission, SLAM, Nav2, etc.)
- Both on same network with same ROS_DOMAIN_ID

Requirements (install on PC):
    pip install python-telegram-bot pillow numpy

Usage:
    # On Pi - start the robot:
    ros2 launch clean_bot_mission cleaning_mission.launch.py
    
    # On PC - start the Telegram bridge:
    export ROS_DOMAIN_ID=0  # Same as Pi
    python3 telegram_bridge.py

Telegram Commands:
    /start - Show available commands
    /scan - Start exploration/scanning
    /stopscan - Stop scanning
    /clean - Start cleaning
    /stopclean - Stop cleaning
    /home - Return to home position
    /reset - Reset mission to initial state
    /pause - Pause current operation
    /resume - Resume from pause
    /status - Get current mission state
    /map - Get map image with robot position
    /help - Show help

Author: Clean Bot Team
"""

import os
import sys
import logging
import asyncio
import io
import threading
import numpy as np
from PIL import Image, ImageDraw, ImageFont

# --- Telegram Imports ---
try:
    from telegram import Update, BotCommand
    from telegram.ext import ApplicationBuilder, ContextTypes, CommandHandler
except ImportError:
    print("❌ Missing telegram library. Install with: pip install python-telegram-bot")
    sys.exit(1)

# --- ROS 2 Imports ---
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from rclpy.executors import MultiThreadedExecutor
    from std_msgs.msg import String
    from nav_msgs.msg import OccupancyGrid
    from tf2_ros import Buffer, TransformListener
except ImportError:
    print("❌ ROS2 not found. Make sure to source ROS2 setup.bash")
    sys.exit(1)

# ==================== USER SETTINGS ====================
TELEGRAM_TOKEN = '7926323441:AAEBdGLAdAkr1KZbVOrx10Aw_pjOTSFZa3o'  # Your bot token from BotFather
ALLOWED_USER_IDS = None  # Set to [123456789] to restrict access, or None for all users
# =======================================================

logging.basicConfig(
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    level=logging.INFO
)
logger = logging.getLogger(__name__)


class RobotBridgeNode(Node):
    """ROS2 Node that bridges Telegram commands to the robot."""
    
    def __init__(self):
        super().__init__('telegram_bridge')
        
        # Publishers
        self.cmd_pub = self.create_publisher(String, 'mission_command', 10)
        
        # QoS for map (transient local, reliable)
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE
        )
        
        # Subscribers
        self.state_sub = self.create_subscription(
            String, 'mission_state', self.state_callback, 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, map_qos)
        self.exploration_state_sub = self.create_subscription(
            String, 'exploration_state', self.exploration_state_callback, 10)
        self.coverage_state_sub = self.create_subscription(
            String, 'coverage_state', self.coverage_state_callback, 10)
        
        # TF for robot position
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # State
        self.mission_state = "UNKNOWN"
        self.exploration_state = "UNKNOWN"
        self.coverage_state = "UNKNOWN"
        self.map_data = None
        self.last_map_time = None
        
        self.get_logger().info('🤖 Telegram Bridge Node initialized')
        self.get_logger().info('   Publishing to: /mission_command')
        self.get_logger().info('   Subscribing to: /mission_state, /map, /exploration_state, /coverage_state')

    def state_callback(self, msg: String):
        self.mission_state = msg.data

    def exploration_state_callback(self, msg: String):
        self.exploration_state = msg.data

    def coverage_state_callback(self, msg: String):
        self.coverage_state = msg.data

    def map_callback(self, msg: OccupancyGrid):
        self.map_data = msg
        self.last_map_time = self.get_clock().now()

    def send_command(self, command: str):
        """Send a command to the robot."""
        msg = String()
        msg.data = command
        self.cmd_pub.publish(msg)
        self.get_logger().info(f'📤 Sent command: {command}')

    def get_robot_position(self):
        """Get robot position from TF."""
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            return (
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.rotation.z,  # Simplified - just z component
                trans.transform.rotation.w
            )
        except Exception as e:
            self.get_logger().debug(f'Could not get TF: {e}')
            return None

    def create_map_image(self):
        """Create a map image with robot position."""
        if self.map_data is None:
            return None, "No map data available"
        
        msg = self.map_data
        width = msg.info.width
        height = msg.info.height
        res = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        
        # Convert to numpy array
        data = np.array(msg.data).reshape((height, width))
        
        # Create RGB image
        # -1 (unknown) = gray, 0 (free) = white, 100 (occupied) = black
        img_pixels = np.zeros((height, width, 3), dtype=np.uint8)
        img_pixels[data == -1] = [128, 128, 128]  # Unknown - gray
        img_pixels[data == 0] = [255, 255, 255]   # Free - white
        img_pixels[data >= 50] = [0, 0, 0]        # Occupied - black
        # Partially occupied (1-49) - gradient
        for val in range(1, 50):
            img_pixels[data == val] = [255 - val*4, 255 - val*4, 255 - val*4]
        
        image = Image.fromarray(img_pixels, mode='RGB')
        draw = ImageDraw.Draw(image)
        
        # Get robot position
        robot_pos = self.get_robot_position()
        robot_info = ""
        
        if robot_pos:
            rx, ry, rz, rw = robot_pos
            # Convert world coords to pixel coords
            px = (rx - origin_x) / res
            py = (ry - origin_y) / res
            
            # Draw robot as a circle with direction indicator
            radius = max(5, int(0.18 / res))  # Robot radius in pixels
            
            # Robot body (red circle)
            draw.ellipse(
                (px - radius, py - radius, px + radius, py + radius),
                fill=(255, 0, 0),
                outline=(200, 0, 0),
                width=2
            )
            
            # Direction indicator (calculate yaw from quaternion)
            import math
            yaw = math.atan2(2.0 * rw * rz, 1.0 - 2.0 * rz * rz)
            arrow_len = radius * 2
            arrow_x = px + arrow_len * math.cos(yaw)
            arrow_y = py + arrow_len * math.sin(yaw)
            draw.line((px, py, arrow_x, arrow_y), fill=(255, 255, 0), width=3)
            
            robot_info = f"Robot at ({rx:.2f}, {ry:.2f})"
        else:
            robot_info = "Robot position unknown"
        
        # Flip image (ROS origin is bottom-left, image origin is top-left)
        image = image.transpose(Image.FLIP_TOP_BOTTOM)
        
        # Scale up if image is small
        min_size = 400
        if width < min_size or height < min_size:
            scale = max(min_size / width, min_size / height)
            new_size = (int(width * scale), int(height * scale))
            image = image.resize(new_size, Image.NEAREST)
        
        return image, robot_info


# Global ROS node
ros_node: RobotBridgeNode = None
ros_executor = None
ros_thread = None


def ros_spin_thread():
    """Thread function to spin ROS2."""
    global ros_executor
    try:
        ros_executor.spin()
    except Exception as e:
        logger.error(f"ROS spin error: {e}")


def init_ros():
    """Initialize ROS2 node and start spinning in background."""
    global ros_node, ros_executor, ros_thread
    
    if not rclpy.ok():
        rclpy.init()
    
    ros_node = RobotBridgeNode()
    ros_executor = MultiThreadedExecutor()
    ros_executor.add_node(ros_node)
    
    ros_thread = threading.Thread(target=ros_spin_thread, daemon=True)
    ros_thread.start()
    
    logger.info("ROS2 node started in background thread")


# ==================== TELEGRAM HANDLERS ====================

async def check_auth(update: Update) -> bool:
    """Check if user is authorized."""
    user_id = update.effective_user.id
    if ALLOWED_USER_IDS and user_id not in ALLOWED_USER_IDS:
        await update.message.reply_text(f"⛔ Access denied. Your ID: {user_id}")
        return False
    return True


async def cmd_start(update: Update, context: ContextTypes.DEFAULT_TYPE):
    """Show welcome message and available commands."""
    if not await check_auth(update):
        return
    
    welcome = """
🤖 *Clean Bot Controller*

Available commands:

*Scanning (Exploration):*
/scan - Start scanning/exploring
/stopscan - Stop scanning

*Cleaning (Coverage):*
/clean - Start cleaning
/stopclean - Stop cleaning

*Control:*
/home - Return to home position
/reset - Reset to initial state
/pause - Pause current operation
/resume - Resume from pause

*Status:*
/status - Get current state
/map - Get map with robot position

/help - Show this message
    """
    await update.message.reply_text(welcome, parse_mode='Markdown')


async def cmd_scan(update: Update, context: ContextTypes.DEFAULT_TYPE):
    """Start scanning/exploration."""
    if not await check_auth(update):
        return
    ros_node.send_command('start_scan')
    await update.message.reply_text('🔍 Starting scan...\nRobot will explore the environment.')


async def cmd_stopscan(update: Update, context: ContextTypes.DEFAULT_TYPE):
    """Stop scanning."""
    if not await check_auth(update):
        return
    ros_node.send_command('stop_scan')
    await update.message.reply_text('🛑 Stopping scan...\nWaiting for clean command.')


async def cmd_clean(update: Update, context: ContextTypes.DEFAULT_TYPE):
    """Start cleaning/coverage."""
    if not await check_auth(update):
        return
    ros_node.send_command('start_clean')
    await update.message.reply_text('🧹 Starting cleaning...\nRobot will cover all free space.')


async def cmd_stopclean(update: Update, context: ContextTypes.DEFAULT_TYPE):
    """Stop cleaning."""
    if not await check_auth(update):
        return
    ros_node.send_command('stop_clean')
    await update.message.reply_text('🛑 Stopping cleaning...')


async def cmd_home(update: Update, context: ContextTypes.DEFAULT_TYPE):
    """Return to home position."""
    if not await check_auth(update):
        return
    ros_node.send_command('go_home')
    await update.message.reply_text('🏠 Returning home...')


async def cmd_reset(update: Update, context: ContextTypes.DEFAULT_TYPE):
    """Reset mission: Stop robot and return to initial state."""
    if not await check_auth(update):
        return
    ros_node.send_command('reset')
    await update.message.reply_text('🛑 Stopping robot...\n🔄 Mission reset to initial state.\nReady for commands.')


async def cmd_pause(update: Update, context: ContextTypes.DEFAULT_TYPE):
    """Pause current operation."""
    if not await check_auth(update):
        return
    ros_node.send_command('pause')
    await update.message.reply_text('⏸️ Paused.')


async def cmd_resume(update: Update, context: ContextTypes.DEFAULT_TYPE):
    """Resume from pause."""
    if not await check_auth(update):
        return
    ros_node.send_command('resume')
    await update.message.reply_text('▶️ Resuming...')


async def cmd_status(update: Update, context: ContextTypes.DEFAULT_TYPE):
    """Get current mission state."""
    if not await check_auth(update):
        return
    
    # State emoji mapping
    state_emoji = {
        'WAITING_FOR_SCAN': '⏳',
        'EXPLORING': '🔍',
        'WAITING_FOR_CLEAN': '⏳',
        'COVERAGE': '🧹',
        'RETURNING': '🏠',
        'COMPLETE': '✅',
        'PAUSED': '⏸️',
        'ERROR': '❌',
        'UNKNOWN': '❓',
        'IDLE': '💤',
        'RUNNING': '🏃',
        'STOPPED': '🛑',
    }
    
    mission_emoji = state_emoji.get(ros_node.mission_state, '❓')
    explore_emoji = state_emoji.get(ros_node.exploration_state, '❓')
    coverage_emoji = state_emoji.get(ros_node.coverage_state, '❓')
    
    # Robot position
    pos = ros_node.get_robot_position()
    pos_str = f"({pos[0]:.2f}, {pos[1]:.2f})" if pos else "Unknown"
    
    # Map info
    map_info = "Not received"
    if ros_node.map_data:
        m = ros_node.map_data.info
        map_info = f"{m.width}x{m.height} @ {m.resolution:.3f}m/px"
    
    status = f"""
📊 *Robot Status*

*Mission State:* {mission_emoji} {ros_node.mission_state}
*Exploration:* {explore_emoji} {ros_node.exploration_state}
*Coverage:* {coverage_emoji} {ros_node.coverage_state}

*Position:* {pos_str}
*Map:* {map_info}
    """
    await update.message.reply_text(status, parse_mode='Markdown')


async def cmd_map(update: Update, context: ContextTypes.DEFAULT_TYPE):
    """Get map image with robot position."""
    if not await check_auth(update):
        return
    
    await update.message.reply_text('🗺️ Generating map...')
    
    # Wait a bit for fresh data
    await asyncio.sleep(0.5)
    
    image, info = ros_node.create_map_image()
    
    if image is None:
        await update.message.reply_text(f'❌ {info}')
        return
    
    # Save to bytes
    bio = io.BytesIO()
    bio.name = 'map.png'
    image.save(bio, 'PNG')
    bio.seek(0)
    
    # State info for caption
    caption = f"""
🗺️ *Map*
{info}
State: {ros_node.mission_state}
    """
    
    await update.message.reply_photo(photo=bio, caption=caption, parse_mode='Markdown')


async def cmd_help(update: Update, context: ContextTypes.DEFAULT_TYPE):
    """Show help."""
    await cmd_start(update, context)


async def post_init(application):
    """Set up bot commands menu."""
    commands = [
        BotCommand("start", "Show welcome and commands"),
        BotCommand("scan", "Start exploration"),
        BotCommand("stopscan", "Stop exploration"),
        BotCommand("clean", "Start cleaning"),
        BotCommand("stopclean", "Stop cleaning"),
        BotCommand("home", "Return to home"),
        BotCommand("reset", "Reset mission"),
        BotCommand("pause", "Pause operation"),
        BotCommand("resume", "Resume operation"),
        BotCommand("status", "Get current state"),
        BotCommand("map", "Get map image"),
        BotCommand("help", "Show help"),
    ]
    await application.bot.set_my_commands(commands)


def main():
    """Main entry point."""
    print("=" * 60)
    print("🤖 Clean Bot Telegram Bridge")
    print("=" * 60)
    print()
    print("This script runs on your PC and communicates with the robot")
    print("via ROS2 over the network.")
    print()
    print("Make sure:")
    print("  1. Robot is running: ros2 launch clean_bot_mission cleaning_mission.launch.py")
    print("  2. Same ROS_DOMAIN_ID on both machines")
    print("  3. Network allows ROS2 DDS traffic (multicast)")
    print()
    
    # Initialize ROS2
    print("Initializing ROS2...")
    init_ros()
    
    # Give ROS time to discover topics
    import time
    time.sleep(2.0)
    
    # Build Telegram application
    print("Starting Telegram bot...")
    application = ApplicationBuilder().token(TELEGRAM_TOKEN).post_init(post_init).build()
    
    # Add handlers
    application.add_handler(CommandHandler('start', cmd_start))
    application.add_handler(CommandHandler('scan', cmd_scan))
    application.add_handler(CommandHandler('stopscan', cmd_stopscan))
    application.add_handler(CommandHandler('clean', cmd_clean))
    application.add_handler(CommandHandler('stopclean', cmd_stopclean))
    application.add_handler(CommandHandler('home', cmd_home))
    application.add_handler(CommandHandler('reset', cmd_reset))
    application.add_handler(CommandHandler('pause', cmd_pause))
    application.add_handler(CommandHandler('resume', cmd_resume))
    application.add_handler(CommandHandler('status', cmd_status))
    application.add_handler(CommandHandler('map', cmd_map))
    application.add_handler(CommandHandler('help', cmd_help))
    
    print()
    print("=" * 60)
    print("✅ Bot is running! Send /start to your bot on Telegram.")
    print("=" * 60)
    
    # Run bot
    application.run_polling()


if __name__ == '__main__':
    main()
