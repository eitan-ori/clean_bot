#!/bin/bash
# =============================================================================
# Script to fix serial port access for LIDAR on Raspberry Pi
# Run with: sudo bash fix_serial_ports.sh
# =============================================================================

set -euo pipefail

echo "=========================================="
echo "🔧 Serial Port Fix Script for LIDAR"
echo "=========================================="
echo ""

# Check what's using the serial ports
echo "📍 Current serial ports:"
ls -la /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "  No USB serial ports found!"
echo ""

echo "📍 Checking what processes are using serial ports..."
for port in /dev/ttyUSB* /dev/ttyACM*; do
    if [ -e "$port" ]; then
        echo "  $port:"
        fuser -v "$port" 2>/dev/null || echo "    (no process using it)"
    fi
done
echo ""

# =============================================================================
# 1. Stop and disable ModemManager (most common culprit)
# =============================================================================
echo "🛑 Stopping ModemManager..."
sudo systemctl stop ModemManager 2>/dev/null || true
sudo systemctl disable ModemManager 2>/dev/null || true
sudo systemctl mask ModemManager 2>/dev/null || true
echo "   Done (ModemManager disabled)"

# =============================================================================
# 2. Stop and disable brltty (Braille TTY - grabs USB serial ports)
# =============================================================================
echo "🛑 Stopping brltty..."
sudo systemctl stop brltty-udev.service 2>/dev/null || true
sudo systemctl stop brltty.service 2>/dev/null || true
sudo systemctl disable brltty-udev.service 2>/dev/null || true
sudo systemctl disable brltty.service 2>/dev/null || true
sudo systemctl mask brltty-udev.service 2>/dev/null || true
sudo systemctl mask brltty.service 2>/dev/null || true

# Also remove brltty udev rules
if [ -f /usr/lib/udev/rules.d/85-brltty.rules ]; then
    echo "   Removing brltty udev rules..."
    sudo mv /usr/lib/udev/rules.d/85-brltty.rules /usr/lib/udev/rules.d/85-brltty.rules.disabled 2>/dev/null || true
fi
echo "   Done (brltty disabled)"

# =============================================================================
# 3. Stop and disable gpsd (GPS daemon)
# =============================================================================
echo "🛑 Stopping gpsd..."
sudo systemctl stop gpsd.socket 2>/dev/null || true
sudo systemctl stop gpsd.service 2>/dev/null || true
sudo systemctl disable gpsd.socket 2>/dev/null || true
sudo systemctl disable gpsd.service 2>/dev/null || true
echo "   Done (gpsd disabled)"

# =============================================================================
# 4. Create udev rules to give proper permissions and prevent grabbing
# =============================================================================
echo "📝 Creating udev rules for LIDAR..."

# Create udev rule for RPLidar (common USB vendor IDs)
sudo tee /etc/udev/rules.d/99-rplidar.rules > /dev/null << 'EOF'
# RPLidar / SLLidar - Silicon Labs CP210x
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", GROUP="dialout", SYMLINK+="rplidar", SYMLINK+="lidar"

# RPLidar - Prolific PL2303
SUBSYSTEM=="tty", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", MODE="0666", GROUP="dialout"

# FTDI serial adapters
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", MODE="0666", GROUP="dialout"

# CH340/CH341 serial adapters
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", MODE="0666", GROUP="dialout"

# Generic - give all ttyUSB ports proper permissions
KERNEL=="ttyUSB[0-9]*", MODE="0666", GROUP="dialout"
KERNEL=="ttyACM[0-9]*", MODE="0666", GROUP="dialout"

# Prevent ModemManager from grabbing these ports
ATTRS{idVendor}=="10c4", ENV{ID_MM_DEVICE_IGNORE}="1"
ATTRS{idVendor}=="067b", ENV{ID_MM_DEVICE_IGNORE}="1"
ATTRS{idVendor}=="0403", ENV{ID_MM_DEVICE_IGNORE}="1"
ATTRS{idVendor}=="1a86", ENV{ID_MM_DEVICE_IGNORE}="1"
EOF

echo "   Created /etc/udev/rules.d/99-rplidar.rules"

# =============================================================================
# 5. Reload udev rules
# =============================================================================
echo "🔄 Reloading udev rules..."
sudo udevadm control --reload-rules
sudo udevadm trigger
echo "   Done"

# =============================================================================
# 6. Set permissions on current ports
# =============================================================================
echo "🔓 Setting permissions on serial ports..."
sudo chmod 666 /dev/ttyUSB* 2>/dev/null || true
sudo chmod 666 /dev/ttyACM* 2>/dev/null || true
echo "   Done"

# =============================================================================
# 7. Show results
# =============================================================================
echo ""
echo "=========================================="
echo "✅ Fix applied! Current state:"
echo "=========================================="
echo ""
echo "📍 Serial ports:"
ls -la /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "  No ports found - replug the LIDAR!"
echo ""

echo "📍 USB devices:"
lsusb | grep -i -E "serial|cp210|uart|ch34|ftdi|pl2303|silicon" || echo "  No known serial adapters found"
echo ""

echo "📍 Disabled services:"
for svc in ModemManager brltty brltty-udev gpsd; do
    status=$(systemctl is-enabled $svc 2>/dev/null || echo "not installed")
    echo "  $svc: $status"
done
echo ""

echo "=========================================="
echo "🔄 IMPORTANT: Unplug and replug the LIDAR!"
echo "=========================================="
echo ""
echo "Then test with:"
echo "  ros2 run clean_bot_hardware check_lidar        # prefers /dev/lidar if present"
echo "  ros2 run clean_bot_hardware check_lidar /dev/lidar"
echo ""
