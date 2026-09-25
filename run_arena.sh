#!/usr/bin/env bash
# Script to launch the NIDAR AirMouse 3D Arena in Gazebo Sim

set -e

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORLD_FILE="${WORKSPACE_DIR}/src/nidar_gazebo/worlds/nidar_arena.sdf"

# Source ROS 2 and Gazebo environment
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
fi

if [ -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
    source "${WORKSPACE_DIR}/install/setup.bash"
fi

# Set model search path for custom drone & survivor models
export GZ_SIM_RESOURCE_PATH="${WORKSPACE_DIR}/src/nidar_gazebo/models:${GZ_SIM_RESOURCE_PATH}"

# Detect if display is available
if [ -z "$DISPLAY" ]; then
    export DISPLAY=":0"
fi
if [ -z "$XAUTHORITY" ] && [ -f "/run/user/1000/gdm/Xauthority" ]; then
    export XAUTHORITY="/run/user/1000/gdm/Xauthority"
fi

echo "=========================================================="
echo "   NIDAR AirMouse - 3D Indoor Arena Simulation"
echo "=========================================================="
echo "Arena Size  : 15m x 15m modular indoor maze"
echo "Corridors   : >= 1.0m uniform clear width"
echo "Rooms       : Standard 2m x 2m (Rooms A through F)"
echo "Ceiling Net : 8 feet (2.44m) covered structure"
echo "Survivors   : 12 dummy targets placed in rooms"
echo "Launch Area : 2ft x 2ft green pad at (-6, -6)"
echo "=========================================================="

if [ "$1" == "--headless" ] || [ "$1" == "-s" ]; then
    echo "Starting Gazebo Sim in HEADLESS (server-only) mode..."
    gz sim -r -s "${WORLD_FILE}"
else
    echo "Starting Gazebo Sim with FULL 3D GUI..."
    gz sim -r "${WORLD_FILE}"
fi
