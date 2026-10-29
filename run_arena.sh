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
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json

# Clean up any previous Gazebo instance
pkill -9 -f 'gz sim' 2>/dev/null || true

HEADLESS="false"
if [ -z "$DISPLAY" ]; then
    HEADLESS="true"
fi

for arg in "$@"; do
  case "$arg" in
    --headless|-s)
      HEADLESS="true"
      ;;
    --gui|-g)
      HEADLESS="false"
      ;;
  esac
done

# If GUI mode is requested from an SSH session, forward to the laptop's active display
if [ "$HEADLESS" == "false" ]; then
    if [ -z "$DISPLAY" ] && [ -e "/tmp/.X11-unix/X0" ]; then
        export DISPLAY=":0"
        MUTTER_AUTH=$(ls /run/user/1000/.mutter-Xwaylandauth.* 2>/dev/null | head -n 1)
        if [ -n "$MUTTER_AUTH" ]; then
            export XAUTHORITY="$MUTTER_AUTH"
        fi
        export WAYLAND_DISPLAY="wayland-0"
        export XDG_RUNTIME_DIR="/run/user/1000"
    fi
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
echo "Mode        : Headless: ${HEADLESS}"
echo "=========================================================="

if [ "$HEADLESS" == "true" ]; then
    echo "Starting Gazebo Sim in HEADLESS (server-only) mode..."
    gz sim -r -s "${WORLD_FILE}"
else
    echo "Starting Gazebo Sim with FULL 3D GUI..."
    gz sim -r "${WORLD_FILE}"
fi
