#!/bin/bash

# --- הגדרות ---
PI_USER="pi_moobot"
PI_HOST="10.44.160.125"
MY_DOMAIN_ID=42
# שימוש ב-CycloneDDS לפתרון בעיות רשת
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=$MY_DOMAIN_ID
# --- הוספה: נתיב לקובץ ההגדרות במחשב המקומי ---
export CYCLONEDDS_URI=file:///root/cyclonedds.xml

# הפקודה המלאה לפיי: מגדירים את ה-RMW וה-ID לפני הרצת ה-Node
# --- שינוי: הוספת CYCLONEDDS_URI גם בפאי ---
PI_LIDAR_CMD="export CYCLONEDDS_URI=file:///home/pi_moobot/cyclonedds.xml && export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && export ROS_DOMAIN_ID=$MY_DOMAIN_ID && source /opt/ros/humble/setup.bash && source ~/robot_ws/install/setup.bash && ros2 run rplidar_ros rplidar_node --ros-args -p serial_port:=/dev/ttyUSB0 -p serial_baudrate:=115200 -p frame_id:=laser"

echo "🚀 Starting RPLidar system..."
# אנחנו משתמשים ב-bash -c כדי להבטיח שכל משתני הסביבה נתפסים
ssh $PI_USER@$PI_HOST "bash -c '$PI_LIDAR_CMD'" &
SSH_PID=$!

echo "⏳ Waiting for LiDAR to initialize..."
sleep 5

# בדיקה האם הטופיק קיים
echo "🔍 Checking for /scan topic..."
# לולאת בדיקה קצרה של 10 שניות
count=0
while [ $count -lt 10 ]; do
    if ros2 topic list | grep -q "/scan"; then
        echo "✅ LiDAR is publishing! (/scan found)"
        break
    fi
    sleep 1
    ((count++))
    echo -n "."
done

if [ $count -eq 10 ]; then
     echo ""
     echo "⚠️  Warning: /scan topic NOT found. Still launching RViz just in case..."
fi

# התחלת RViz
echo "🖥️  Starting RViz..."
ros2 launch clean_bot_hardware rviz_lidar.launch.py &
RVIZ_PID=$!

echo ""
echo "✅ System started! (PID: $SSH_PID)"
echo "Press Ctrl+C to stop everything"

cleanup() {
    echo ""
    echo "🛑 Stopping system..."
    kill $RVIZ_PID 2>/dev/null
    ssh $PI_USER@$PI_HOST "pkill -f rplidar_node" 2>/dev/null
    exit 0
}

trap cleanup SIGINT SIGTERM

wait