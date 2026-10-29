#!/usr/bin/env bash
# Source ROS 2 and workspace
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
fi
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
    source "${WORKSPACE_DIR}/install/setup.bash"
fi

CMD="${1:-help}"

case "$CMD" in
  takeoff)
    echo "Sending AUTONOMOUS TAKEOFF command (target: 1.20m)..."
    ros2 topic pub --once /drone/cmd std_msgs/msg/String "{data: 'takeoff'}"
    ;;
  explore)
    echo "Sending AUTONOMOUS EXPLORE MAZE command..."
    ros2 topic pub --once /drone/cmd std_msgs/msg/String "{data: 'explore'}"
    ;;
  hover)
    echo "Sending HOVER command..."
    ros2 topic pub --once /drone/cmd std_msgs/msg/String "{data: 'hover'}"
    ;;
  rth)
    echo "Sending RETURN TO HOME / EXIT POINT (RTH) command..."
    ros2 topic pub --once /drone/cmd std_msgs/msg/String "{data: 'rth'}"
    ;;
  land)
    echo "Sending PRECISION AUTO-LAND command..."
    ros2 topic pub --once /drone/cmd std_msgs/msg/String "{data: 'land'}"
    ;;
  stop|abort)
    echo "Sending EMERGENCY STOP command..."
    ros2 topic pub --once /drone/cmd std_msgs/msg/String "{data: 'stop'}"
    ;;
  status)
    echo "Listening to /drone/status... (Press Ctrl+C to exit)"
    ros2 topic echo /drone/status
    ;;
  survivors)
    echo "Listening to detected survivors (/drone/survivors)... (Press Ctrl+C to exit)"
    ros2 topic echo /drone/survivors
    ;;
  tf02)
    echo "Listening to LiDAR 2 (Downward ToF Altimeter /drone/tf02_pro/scan)... (Press Ctrl+C to exit)"
    ros2 topic echo /drone/tf02_pro/scan
    ;;
  scan)
    echo "Listening to LiDAR 1 (Horizontal 360 2D LiDAR /scan)... (Press Ctrl+C to exit)"
    ros2 topic echo /scan
    ;;
  map)
    echo "Checking 2D SLAM Map topic (/map)... (Press Ctrl+C to exit)"
    ros2 topic echo /map --no-arr
    ;;
  *)
    echo "=========================================================================="
    echo "     NIDAR AirMouse Dual-LiDAR Flight Control & Inspection Utility"
    echo "=========================================================================="
    echo "Usage: ./test_autonomous.sh [command]"
    echo ""
    echo "Available commands:"
    echo "  takeoff    Climb to 1.20m cruising altitude using Downward ToF LiDAR"
    echo "  explore    Autonomous corridor centering & maze exploration using 2D LiDAR"
    echo "  hover      Hold current position and altitude"
    echo "  rth        Autonomous Return-To-Home to designated entry/exit pad"
    echo "  land       Gentle precision landing using ToF altimeter"
    echo "  stop       Emergency hover / stop"
    echo "  status     Monitor drone autonomous state, coordinates & clearances"
    echo "  survivors  Echo live detected survivors and grid box coordinates"
    echo "  tf02       Echo LiDAR 2 (Downward ToF ground distance)"
    echo "  scan       Echo LiDAR 1 (Horizontal 360 LaserScan)"
    echo "  map        Inspect 2D SLAM OccupancyGrid header metadata"
    echo "=========================================================================="
    ;;
esac
