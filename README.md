# Nidar_Ros

ROS 2 & Gazebo Simulation Workspace for the **NIDAR AirMouse 3D Arena**.

## Overview

This workspace contains the simulation environment and configuration for the NIDAR AirMouse drone in Gazebo Sim and ROS 2.

### Features
- **3D Indoor Arena**: 15m x 15m modular maze environment with corridors (>= 1.0m width), standard 2m x 2m rooms (Rooms A through F), ceiling net (8ft / 2.44m), launch area pad, and survivor targets.
- **Multicopter Simulation**: Inbuilt PX5 / X3 multicopter model configured with sensors, actuators, and Gazebo twist velocity control.
- **Launch & Test Utilities**:
  - `run_arena.sh`: Launch Gazebo Sim with full 3D GUI or headless server mode.
  - `test_drone.sh`: Flight tester utility script for sending takeoff, hover, translational, yaw, and landing commands.
  - `src/nidar_gazebo/launch/nidar_sim.launch.py`: ROS 2 launch file integration with `ros_gz_bridge`.

---

## Directory Structure

```
├── .gitignore
├── README.md
├── run_arena.sh                  # Launches the 3D arena in Gazebo Sim
├── test_drone.sh                 # Flight test command utility
└── src
    └── nidar_gazebo
        ├── CMakeLists.txt
        ├── package.xml
        ├── config
        │   └── ros_gz_bridge.yaml
        ├── launch
        │   └── nidar_sim.launch.py
        ├── models
        │   ├── airmouse_drone
        │   └── px5_drone
        └── worlds
            ├── first_world.sdf
            └── nidar_arena.sdf
```

---

## Getting Started

### Prerequisites
- ROS 2 (e.g. Jazzy)
- Gazebo Sim (`gz-sim`)
- `ros_gz_bridge`

### Build the Workspace

From the workspace root:

```bash
colcon build --symlink-install
source install/setup.bash
```

### Launch Arena Simulation

Run the arena in GUI mode:
```bash
./run_arena.sh
```

Or run headless:
```bash
./run_arena.sh --headless
```

### Drone Control Testing

With the simulation running, in a separate terminal:
```bash
# Take off into hover
./test_drone.sh takeoff

# Hold hover
./test_drone.sh hover

# Move forward at 0.6 m/s
./test_drone.sh forward

# Yaw rotation
./test_drone.sh turn_left

# Land drone
./test_drone.sh land

# Stream live drone odometry
./test_drone.sh odom
```
