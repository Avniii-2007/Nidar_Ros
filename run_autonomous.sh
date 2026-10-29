#!/usr/bin/env bash
# Launch script for NIDAR AirMouse Dual-LiDAR Autonomous Drone & Arena Simulation
set -e

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Source ROS 2
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
fi

# Source this workspace
if [ -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
    source "${WORKSPACE_DIR}/install/setup.bash"
else
    echo "Notice: Building workspace with colcon..."
    cd "${WORKSPACE_DIR}" && colcon build --symlink-install --event-handlers console_direct+
    source "${WORKSPACE_DIR}/install/setup.bash"
fi

export GZ_SIM_RESOURCE_PATH="${WORKSPACE_DIR}/src/nidar_gazebo/models:${GZ_SIM_RESOURCE_PATH}"
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json

HEADLESS="true"
AUTO_START="true"
RVIZ="false"

for arg in "$@"; do
  case "$arg" in
    --headless|-s)
      HEADLESS="true"
      ;;
    --gui|-g)
      HEADLESS="false"
      ;;
    --auto-start|-a)
      AUTO_START="true"
      ;;
    --manual|--no-auto-start|-m)
      AUTO_START="false"
      ;;
    --rviz|-r)
      RVIZ="true"
      ;;
    --help|-h)
      echo "Usage: ./run_autonomous.sh [options]"
      echo "Options:"
      echo "  --headless, -s         Run Gazebo Sim in headless mode (server only, default on SSH)"
      echo "  --gui, -g              Run Gazebo Sim with 3D GUI window on laptop display"
      echo "  --auto-start, -a       Automatically initiate takeoff and maze exploration (DEFAULT: true)"
      echo "  --manual, -m           Start in IDLE state (manual takeoff via GCS or terminal)"
      echo "  --rviz, -r             Launch RViz2 3D/2D visualization"
      exit 0
      ;;
  esac
done

# If GUI mode is requested from an SSH session, forward to the laptop's active display
if [ "$HEADLESS" == "false" ] || [ "$RVIZ" == "true" ]; then
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

# Clean up any lingering simulation processes before launch
cleanup_simulation() {
    pkill -9 -f 'parameter_bridge' 2>/dev/null || true
    pkill -9 -f 'gz sim' 2>/dev/null || true
    pkill -9 -f 'drone_autonomy_controller' 2>/dev/null || true
    pkill -9 -f 'survivor_detector' 2>/dev/null || true
    pkill -9 -f 'mission_planner_gcs' 2>/dev/null || true
    pkill -9 -f 'async_slam_toolbox_node' 2>/dev/null || true
    pkill -9 -f 'static_transform_publisher' 2>/dev/null || true
    pkill -9 -f 'autonomous_drone.launch.py' 2>/dev/null || true
    pkill -9 -f 'nidar_sim.launch.py' 2>/dev/null || true
    sleep 0.5
}

cleanup_simulation
trap cleanup_simulation EXIT INT TERM

HOST_IP=$(hostname -I | awk '{print $1}')

echo "=========================================================================="
echo "          NIDAR AirMouse - Dual-LiDAR Autonomous Drone System"
echo "=========================================================================="
echo " Autonomy  : Dual-LiDAR Pure Navigation & Closed-Loop Altitude Control"
echo "   LiDAR 1 : Horizontal 360° 2D LiDAR (/scan) -> slam_toolbox 2D Mapping"
echo "   LiDAR 2 : Downward ToF LiDAR (/drone/tf02_pro/scan) -> Sub-cm Altitude"
echo " Vision    : Camera Survivor Localisation & Grid Box Tagging (/survivor_markers)"
echo " Mode      : Headless Sim (NVIDIA EGL Hardware Accelerated: ${HEADLESS})"
GCS_PORT="8080"
if ss -tuln | grep -q ":8080 "; then
    GCS_PORT="8081"
    echo " Note      : Port 8080 is currently occupied. GCS will bind to fallback port ${GCS_PORT}."
fi

echo " GCS Server: http://localhost:${GCS_PORT}"
if [ -n "$HOST_IP" ]; then
    echo " Network   : http://${HOST_IP}:${GCS_PORT} (Direct browser access on same network)"
    echo " SSH Tunnel: ssh -L ${GCS_PORT}:localhost:${GCS_PORT} shubham@${HOST_IP}"
fi
echo " Failsafes : 8ft (2.44m) Ceiling Net Limit & Emergency Wall Proximity Buffer"
echo "=========================================================================="

ros2 launch nidar_gazebo autonomous_drone.launch.py \
    headless:="${HEADLESS}" \
    auto_start:="${AUTO_START}" \
    rviz:="${RVIZ}"
