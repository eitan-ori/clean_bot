#!/bin/bash
# Test script for coverage mission with useful debug output
#
# Usage:
#   ./test_coverage.sh           # Normal test
#   ./test_coverage.sh debug     # Full debug output
#   ./test_coverage.sh filter    # Filtered output (only movement)

MODE=${1:-normal}

echo "=========================================="
echo "🧪 Coverage Test Mode: $MODE"
echo "=========================================="

if [ "$MODE" == "debug" ]; then
    # Full debug - see everything
    ros2 launch clean_bot_mission cleaning_mission.launch.py log_level:=debug
    
elif [ "$MODE" == "filter" ]; then
    # Filtered - only movement related logs
    ros2 launch clean_bot_mission cleaning_mission.launch.py log_level:=debug 2>&1 | \
        grep --line-buffered -E "(waypoint|Turn|Drive|Reached|Off course|COMPLETE|error|ERROR|warn|WARN|→|✅|🔄|🚗|↩️|🏁|Starting|Stopping)"
        
else
    # Normal - INFO level
    ros2 launch clean_bot_mission cleaning_mission.launch.py
fi
