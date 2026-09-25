#!/usr/bin/env bash
# Script to send flight test commands to the inbuilt PX5 / X3 multicopter in Gazebo Sim

CMD="${1:-help}"
TOPIC="/X3/gazebo/command/twist"

case "$CMD" in
  takeoff)
    echo "Sending TAKEOFF command (climbing at +0.8 m/s)..."
    gz topic -t "$TOPIC" -m gz.msgs.Twist -p "linear: {x: 0.0, y: 0.0, z: 0.8}, angular: {z: 0.0}"
    ;;
  hover)
    echo "Sending HOVER command (zero velocity hold)..."
    gz topic -t "$TOPIC" -m gz.msgs.Twist -p " "
    ;;
  forward)
    SPEED="${2:-0.6}"
    echo "Sending FORWARD command (speed: $SPEED m/s)..."
    gz topic -t "$TOPIC" -m gz.msgs.Twist -p "linear: {x: $SPEED, y: 0.0, z: 0.0}, angular: {z: 0.0}"
    ;;
  backward)
    SPEED="${2:-0.6}"
    echo "Sending BACKWARD command (speed: $SPEED m/s)..."
    gz topic -t "$TOPIC" -m gz.msgs.Twist -p "linear: {x: -$SPEED, y: 0.0, z: 0.0}, angular: {z: 0.0}"
    ;;
  left)
    SPEED="${2:-0.5}"
    echo "Sending STRAFE LEFT command (speed: $SPEED m/s)..."
    gz topic -t "$TOPIC" -m gz.msgs.Twist -p "linear: {x: 0.0, y: $SPEED, z: 0.0}, angular: {z: 0.0}"
    ;;
  right)
    SPEED="${2:-0.5}"
    echo "Sending STRAFE RIGHT command (speed: $SPEED m/s)..."
    gz topic -t "$TOPIC" -m gz.msgs.Twist -p "linear: {x: 0.0, y: -$SPEED, z: 0.0}, angular: {z: 0.0}"
    ;;
  turn_left)
    echo "Sending YAW LEFT command (+0.5 rad/s)..."
    gz topic -t "$TOPIC" -m gz.msgs.Twist -p "linear: {z: 0.0}, angular: {z: 0.5}"
    ;;
  turn_right)
    echo "Sending YAW RIGHT command (-0.5 rad/s)..."
    gz topic -t "$TOPIC" -m gz.msgs.Twist -p "linear: {z: 0.0}, angular: {z: -0.5}"
    ;;
  land)
    echo "Sending LAND command (descending at -0.5 m/s)..."
    gz topic -t "$TOPIC" -m gz.msgs.Twist -p "linear: {x: 0.0, y: 0.0, z: -0.5}, angular: {z: 0.0}"
    ;;
  odom)
    echo "Listening to drone odometry (/model/X3/odometry)... Press Ctrl+C to stop."
    gz topic -e -t "/model/X3/odometry"
    ;;
  *)
    echo "=========================================================="
    echo "       Inbuilt PX5 / X3 Multicopter Flight Tester"
    echo "=========================================================="
    echo "Usage: ./test_drone.sh [command]"
    echo ""
    echo "Available commands:"
    echo "  takeoff       Climb up into hover"
    echo "  hover         Hold altitude and position"
    echo "  forward       Move forward in corridor"
    echo "  backward      Move backward"
    echo "  left          Strafe left"
    echo "  right         Strafe right"
    echo "  turn_left     Rotate yaw left"
    echo "  turn_right    Rotate yaw right"
    echo "  land          Gently touch down on floor/pad"
    echo "  odom          Echo live drone position & velocity"
    echo "=========================================================="
    ;;
esac
