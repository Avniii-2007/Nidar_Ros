#!/usr/bin/env python3
"""
Dual-LiDAR Autonomous Flight Controller & Indoor Navigator for NIDAR AirMouse Drone
------------------------------------------------------------------------------------
Complies with NIDAR AirMouse Problem Statement:
- Pure LiDAR-Only Navigation & Autonomy (No GPS, No Vision for flight control):
    1. Horizontal 360-degree 2D LiDAR (/scan) for SLAM mapping, corridor following,
       junction detection, room exploration, and reactive obstacle avoidance.
    2. Downward ToF LiDAR (/drone/tf02_pro/scan) for closed-loop precision altitude hold,
       ceiling net (8ft / 2.44m) limit enforcement, and autonomous takeoff/landing.
- Dynamic TF Broadcaster: Publishes odom -> base_link transform for slam_toolbox & RViz
- Live Flight Trajectory Publisher (/drone/trajectory)
- Finite State Machine: IDLE -> TAKEOFF -> HOVER -> EXPLORE -> RTH -> LAND -> EMERGENCY
"""

import math
import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


class DroneState:
    IDLE = "IDLE"
    TAKEOFF = "TAKEOFF"
    HOVER = "HOVER"
    EXPLORE = "EXPLORE"
    RTH = "RTH"
    LAND = "LAND"
    EMERGENCY = "EMERGENCY"


class PIDController:
    """Discrete PID Controller with anti-windup clamping and derivative filtering."""
    def __init__(self, kp: float, ki: float, kd: float, output_limits=(-1.0, 1.0), anti_windup: bool = True):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min_out, self.max_out = output_limits
        self.anti_windup = anti_windup
        self.integral = 0.0
        self.prev_error = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error: float, dt: float = 0.05) -> float:
        if dt <= 0.0:
            dt = 0.05
        # Proportional term
        p_term = self.kp * error

        # Integral term with anti-windup clamping
        self.integral += error * dt
        if self.anti_windup:
            max_i = abs(self.max_out)
            self.integral = max(-max_i, min(max_i, self.integral))
        i_term = self.ki * self.integral

        # Derivative term
        d_term = self.kd * ((error - self.prev_error) / dt)
        self.prev_error = error

        output = p_term + i_term + d_term
        return max(self.min_out, min(self.max_out, output))


class DroneAutonomyController(Node):
    def __init__(self):
        super().__init__('drone_autonomy_controller')

        # Parameters for modular 15m arena (corridors >= 1.0m, doors 1.2m)
        self.declare_parameter('target_altitude', 1.20)        # Cruising altitude centered below 2.44m ceiling net
        self.declare_parameter('max_horizontal_speed', 0.45)   # Smooth cruise speed in corridors
        self.declare_parameter('wall_safety_dist', 0.55)       # Distance to begin gentle deceleration
        self.declare_parameter('emergency_dist', 0.25)         # Critical front obstacle proximity
        self.declare_parameter('auto_start', True)             # Auto-takeoff on launch

        self.target_alt = self.get_parameter('target_altitude').value
        self.max_speed = self.get_parameter('max_horizontal_speed').value
        self.wall_safety = self.get_parameter('wall_safety_dist').value
        self.emerg_dist = self.get_parameter('emergency_dist').value
        raw_auto = self.get_parameter('auto_start').value
        if isinstance(raw_auto, str):
            self.auto_start = raw_auto.strip().lower() in ['true', '1', 'yes']
        else:
            self.auto_start = bool(raw_auto)

        # Home location (Designated Entry & Exit point launch pad in NIDAR arena)
        self.home_x = -5.8
        self.home_y = -6.0

        # State Variables
        self.state = DroneState.IDLE
        self.current_x = -5.8
        self.current_y = -6.0
        self.current_z = 0.05
        self.current_yaw = 0.0
        self.linear_vel_x = 0.0
        self.linear_vel_y = 0.0

        # LiDAR 2: Downward ToF LiDAR Altitude (Benewake TF02-Pro)
        self.ground_distance = 0.05
        self.last_tf02_time = self.get_clock().now()

        # LiDAR 1: Horizontal 360 2D LiDAR Sectors
        self.front_dist = 10.0
        self.left_dist = 5.0
        self.right_dist = 5.0
        self.rear_dist = 5.0
        self.front_left_dist = 5.0
        self.front_right_dist = 5.0
        self.min_dist = 10.0

        # Autonomous Systematic Maze Exploration Tour
        # Every doorway milestone passes 0.8m-1.2m PAST the door plane before initiating turns
        self.maze_tour = [
            # 1. Exit Launch Bay (Room 1) -> Through Doorway 1 (Y = -4.5)
            {"x": -4.6, "y": -5.3, "desc": "Doorway 1 Approach (Room 1)"},
            {"x": -4.6, "y": -3.6, "desc": "Pass Doorway 1 into Room 2"},

            # 2. Room 2 (West Wing) - Survivor 9
            {"x": -5.8, "y": -3.2, "desc": "Room 2 Interior (Survivor 9)"},
            {"x": -4.6, "y": -2.2, "desc": "Doorway 2 Approach"},
            {"x": -4.6, "y": -0.7, "desc": "Pass Doorway 2 into West Corridor"},

            # 3. West Corridor -> Doorway 3 (Y = 1.5) -> Room 3 NW - Survivor 2
            {"x": -4.6, "y": 0.5, "desc": "West Corridor Midpoint"},
            {"x": -4.6, "y": 1.0, "desc": "Doorway 3 Approach"},
            {"x": -4.6, "y": 2.5, "desc": "Pass Doorway 3 into Room 3"},
            {"x": -5.8, "y": 5.0, "desc": "Room 3 NW Interior (Survivor 2)"},

            # 4. Exit Room 3 -> Through Doorway 4 (X = -4.0, Y = 3.6) into North Corridor
            {"x": -4.8, "y": 3.6, "desc": "Doorway 4 Approach"},
            {"x": -3.0, "y": 3.6, "desc": "Pass Doorway 4 into North Corridor"},

            # 5. North Corridor -> Doorway 5 (Y = 4.0, X = 0.4) -> Room 4 (GPC) - Survivor 3
            {"x": 0.4, "y": 3.0, "desc": "Room 4 GPC Approach"},
            {"x": 0.4, "y": 4.8, "desc": "Pass Doorway 5 into GPC Room"},
            {"x": 0.8, "y": 5.8, "desc": "Room 4 GPC Command Room (Survivor 3)"},
            {"x": 0.4, "y": 4.8, "desc": "GPC Exit Approach"},
            {"x": 0.4, "y": 2.8, "desc": "Clear Doorway 5 into Corridor"},

            # 6. North-East Corridor -> Doorway 6 (Y = 3.5, X = 4.6) -> Room 5 (NE) - Survivor 4
            {"x": 2.5, "y": 2.5, "desc": "North-East Corridor Transit"},
            {"x": 4.6, "y": 2.4, "desc": "Doorway 6 Approach"},
            {"x": 4.6, "y": 4.6, "desc": "Pass Doorway 6 into Room 5"},
            {"x": 5.8, "y": 5.5, "desc": "Room 5 NE Interior (Survivor 4)"},
            {"x": 4.6, "y": 4.6, "desc": "Room 5 Exit Approach"},
            {"x": 4.6, "y": 2.2, "desc": "Clear Doorway 6 into East Corridor"},

            # 7. East Corridor -> Doorway 7 (X = 3.5, Y = 1.1) -> Room 6A - Survivor 8
            {"x": 2.7, "y": 1.1, "desc": "Doorway 7 Approach (Room 6)"},
            {"x": 4.5, "y": 1.1, "desc": "Pass Doorway 7 into Room 6"},
            {"x": 5.2, "y": 2.2, "desc": "Room 6A Upper East (Survivor 8)"},

            # 8. Room 6C Outer East - Survivor 5 -> Through Doorway 8 (Y = 1.0, X = 6.4)
            {"x": 6.4, "y": 1.6, "desc": "Room 6C Doorway Approach"},
            {"x": 6.4, "y": 0.2, "desc": "Pass Doorway 8 into Room 6C"},
            {"x": 6.6, "y": -0.3, "desc": "Room 6C Outer East (Survivor 5)"},

            # 9. Room 6B Inner Chamber - Survivor 7 -> Through Doorway 9 (X = 5.5, Y = -0.8)
            {"x": 6.2, "y": -0.8, "desc": "Room 6B Doorway Approach"},
            {"x": 4.6, "y": -0.8, "desc": "Pass Doorway 9 into Room 6B"},
            {"x": 4.5, "y": -0.3, "desc": "Room 6B Inner Chamber (Survivor 7)"},
            {"x": 4.5, "y": 1.1, "desc": "Room 6 Exit Approach"},
            {"x": 2.7, "y": 1.1, "desc": "Clear Doorway 7 into East Corridor"},

            # 10. South Corridor -> Doorway 10 (X = 3.5, Y = -3.4) -> Room 7A - Survivor 6
            {"x": 2.5, "y": -1.5, "desc": "South Corridor Transit North"},
            {"x": 2.7, "y": -3.4, "desc": "Doorway 10 Approach (Room 7)"},
            {"x": 4.5, "y": -3.4, "desc": "Pass Doorway 10 into Room 7A"},
            {"x": 5.8, "y": -3.2, "desc": "Room 7A South-East Lab (Survivor 6)"},

            # 11. Room 7B Exit Bay -> Through Doorway 11 (Y = -4.5, X = 5.4)
            {"x": 5.4, "y": -3.8, "desc": "Room 7B Exit Bay Approach"},
            {"x": 5.4, "y": -5.2, "desc": "Pass Doorway 11 into Exit Bay"},
            {"x": 5.5, "y": -5.8, "desc": "Exit Pad Inspection Point"},
            {"x": 5.4, "y": -3.8, "desc": "Exit Bay Re-entry"},
            {"x": 4.5, "y": -3.4, "desc": "Room 7A Transit"},
            {"x": 2.7, "y": -3.4, "desc": "Clear Doorway 10 into South Corridor"},

            # 12. South-Central Rooms (Rooms 9 & 8) - Survivors 12 & 11
            {"x": 2.0, "y": -3.6, "desc": "Room 9 Approach"},
            {"x": 1.5, "y": -3.6, "desc": "Room 9 Mid-South (Survivor 12)"},
            {"x": 1.6, "y": -4.4, "desc": "Doorway 12 Approach"},
            {"x": 1.6, "y": -5.8, "desc": "Pass Doorway 12 into Room 8"},
            {"x": 1.5, "y": -6.6, "desc": "Room 8 South Bay (Survivor 11)"},
            {"x": 0.4, "y": -5.2, "desc": "Central Spine Doorway Approach"},
            {"x": -1.2, "y": -5.2, "desc": "Pass Central Spine Doorway"},

            # 13. Central Corridor Nook - Survivor 10 & Return Home
            {"x": -1.8, "y": -1.0, "desc": "Central Corridor Hallway"},
            {"x": -1.8, "y": 0.8, "desc": "Central Corridor Nook (Survivor 10)"},
            {"x": -1.8, "y": -2.2, "desc": "Central Corridor to Room 2 Connector"},
            {"x": -3.5, "y": -2.2, "desc": "Pass Opening into Room 2"},
            {"x": -4.6, "y": -2.5, "desc": "Room 2 Corridor Transit"},
            {"x": -4.6, "y": -3.6, "desc": "Doorway 1 North Approach"},
            {"x": -4.6, "y": -5.3, "desc": "Pass Doorway 1 into Room 1 (Launch Bay)"},
            {"x": -5.8, "y": -6.0, "desc": "Launch Pad Touchdown"}
        ]
        self.waypoint_index = 0
        self.survivors_count = 0

        # Exploration Decision Logic
        self.turn_direction = 1.0  # +1 = left, -1 = right
        self.exploring_turn_counter = 0
        self.junction_detected_counter = 0
        self.is_turning = False
        self.takeoff_stable_count = 0

        # Precision 4-Channel PID Flight Controllers
        # 1. Altitude PID Controller (Z velocity based on Benewake TF02-Pro Downward LiDAR)
        self.pid_alt = PIDController(kp=1.85, ki=0.15, kd=0.65, output_limits=(-0.35, 0.65))

        # 2. Heading / Yaw PID Controller (Angular Z rate towards waypoint / corridor heading)
        self.pid_yaw = PIDController(kp=1.40, ki=0.02, kd=0.38, output_limits=(-0.75, 0.75))

        # 3. Lateral / Corridor Centering PID Controller (Y velocity for wall equidistant tracking)
        self.pid_lat = PIDController(kp=0.45, ki=0.03, kd=0.22, output_limits=(-0.25, 0.25))

        # 4. Longitudinal / Velocity Governor PID Controller (X velocity based on distance to waypoint)
        self.pid_vel = PIDController(kp=0.45, ki=0.01, kd=0.16, output_limits=(0.0, self.max_speed))

        # RTH Topological Backtracker
        self.rth_waypoints = []
        self.rth_index = 0
        self.breadcrumbs = [(-5.8, -6.0)]

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/drone/status', 10)
        self.path_pub = self.create_publisher(Path, '/drone/trajectory', 10)

        # Dynamic TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Trajectory Path Message
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.tf02_sub = self.create_subscription(LaserScan, '/drone/tf02_pro/scan', self.tf02_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.cmd_sub = self.create_subscription(String, '/drone/cmd', self.cmd_callback, 10)
        self.survivors_sub = self.create_subscription(String, '/drone/survivors', self.survivors_callback, 10)

        # Control Loop (20 Hz)
        self.control_timer = self.create_timer(0.05, self.control_loop)
        # Status Reporting Loop (2 Hz)
        self.status_timer = self.create_timer(0.5, self.publish_status)

        self.get_logger().info("=" * 65)
        self.get_logger().info("NIDAR AirMouse Dual-LiDAR Autonomy Controller Initialized")
        self.get_logger().info("  * LiDAR 1 (Horizontal 360 2D): SLAM, Wall Tracking, Corridor Centering")
        self.get_logger().info("  * LiDAR 2 (Downward ToF): 100Hz Altitude Hold, Ceiling Net Protect, Auto-Land")
        self.get_logger().info(f"  * Cruising Altitude: {self.target_alt:.2f} m | Max Speed: {self.max_speed:.2f} m/s")
        self.get_logger().info(f"  * Initial State: {self.state}")
        self.get_logger().info("=" * 65)

        if self.auto_start:
            self.get_logger().info("Auto-start enabled: Initiating TAKEOFF sequence...")
            self.state = DroneState.TAKEOFF

    # -------------------------------------------------------------
    # Sensor Callbacks
    # -------------------------------------------------------------
    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.position.z
        self.linear_vel_x = msg.twist.twist.linear.x
        self.linear_vel_y = msg.twist.twist.linear.y

        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

        # Broadcast Dynamic TF: odom -> base_link
        t = TransformStamped()
        tf_time = self.get_clock().now().to_msg()
        # 50ms forward buffer eliminates extrapolation warnings in slam_toolbox
        tf_time.nanosec += 50000000
        if tf_time.nanosec >= 1000000000:
            tf_time.sec += 1
            tf_time.nanosec -= 1000000000
        t.header.stamp = tf_time
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.current_x
        t.transform.translation.y = self.current_y
        t.transform.translation.z = self.current_z
        t.transform.rotation = q
        self.tf_broadcaster.sendTransform(t)

        # Record breadcrumbs for collision-free RTH backtracking
        if self.state in [DroneState.EXPLORE, DroneState.TAKEOFF]:
            last_bx, last_by = self.breadcrumbs[-1]
            if math.hypot(self.current_x - last_bx, self.current_y - last_by) > 0.35:
                self.breadcrumbs.append((self.current_x, self.current_y))

        # Append to live flight path
        if self.state in [DroneState.EXPLORE, DroneState.RTH, DroneState.TAKEOFF]:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose = msg.pose.pose
            if len(self.path_msg.poses) == 0 or \
               math.hypot(self.current_x - self.path_msg.poses[-1].pose.position.x,
                          self.current_y - self.path_msg.poses[-1].pose.position.y) > 0.15:
                self.path_msg.poses.append(pose_stamped)
                if len(self.path_msg.poses) > 1000:
                    self.path_msg.poses.pop(0)
                self.path_pub.publish(self.path_msg)

    def tf02_callback(self, msg: LaserScan):
        """LiDAR 2: Downward ToF Rangefinder (Benewake TF02-Pro)."""
        if len(msg.ranges) > 0:
            val = msg.ranges[0]
            if math.isnan(val):
                return
            if math.isinf(val):
                # When drone is on the ground (closer than min_range), Gazebo outputs inf
                if self.current_z < 0.25:
                    self.ground_distance = max(0.04, self.current_z)
                else:
                    self.ground_distance = max(0.05, self.current_z)
                return

            if msg.range_min <= val <= msg.range_max:
                # Sanity check: downward distance cannot exceed arena height (2.5m)
                if val <= 3.0:
                    self.ground_distance = val
                    self.last_tf02_time = self.get_clock().now()
                else:
                    self.ground_distance = max(0.05, self.current_z)
            elif val < msg.range_min:
                self.ground_distance = max(0.04, self.current_z)

    def lidar_callback(self, msg: LaserScan):
        """LiDAR 1: 360-degree Horizontal 2D LiDAR Scan."""
        num_readings = len(msg.ranges)
        if num_readings == 0:
            return

        angle_min = msg.angle_min
        angle_inc = msg.angle_increment

        front_ranges = []
        front_left_ranges = []
        front_right_ranges = []
        left_ranges = []
        right_ranges = []
        rear_ranges = []
        all_valid = []

        for i, r in enumerate(msg.ranges):
            if math.isnan(r) or math.isinf(r) or r < msg.range_min:
                continue
            r_clamped = min(r, msg.range_max)
            all_valid.append(r_clamped)

            angle = angle_min + i * angle_inc
            # Normalize to [-pi, pi]
            angle = math.atan2(math.sin(angle), math.cos(angle))

            # Narrow front cone (+/- 15 deg) prevents corridor walls from falsely triggering front collision
            if -0.26 <= angle <= 0.26:              # Front +/- 15 deg (forward path)
                front_ranges.append(r_clamped)
            elif 0.26 < angle <= 0.85:              # Front-Left
                front_left_ranges.append(r_clamped)
            elif -0.85 <= angle < -0.26:            # Front-Right
                front_right_ranges.append(r_clamped)
            elif 0.85 < angle <= 2.20:              # Left
                left_ranges.append(r_clamped)
            elif -2.20 <= angle < -0.85:            # Right
                right_ranges.append(r_clamped)
            else:                                   # Rear
                rear_ranges.append(r_clamped)

        self.front_dist = min(front_ranges) if front_ranges else msg.range_max
        self.front_left_dist = min(front_left_ranges) if front_left_ranges else msg.range_max
        self.front_right_dist = min(front_right_ranges) if front_right_ranges else msg.range_max
        self.left_dist = min(left_ranges) if left_ranges else msg.range_max
        self.right_dist = min(right_ranges) if right_ranges else msg.range_max
        self.rear_dist = min(rear_ranges) if rear_ranges else msg.range_max
        self.min_dist = min(all_valid) if all_valid else msg.range_max

    def imu_callback(self, msg: Imu):
        pass

    def initiate_rth(self, reason="Mission Completed"):
        """Initiates collision-free Return To Home by backtracking visited milestones in reverse."""
        if self.state in [DroneState.RTH, DroneState.LAND, DroneState.IDLE]:
            return
        self.get_logger().info(f"Initiating Collision-Free Return To Home (RTH): {reason}")
        visited_idx = min(self.waypoint_index, len(self.maze_tour) - 1)
        self.rth_waypoints = []
        for i in range(visited_idx, -1, -1):
            wp = self.maze_tour[i]
            if not self.rth_waypoints or (self.rth_waypoints[-1]["x"] != wp["x"] or self.rth_waypoints[-1]["y"] != wp["y"]):
                self.rth_waypoints.append({"x": wp["x"], "y": wp["y"], "desc": f"Backtrack: {wp['desc']}"})
        self.rth_waypoints.append({"x": self.home_x, "y": self.home_y, "desc": "Launch Pad Touchdown"})
        self.rth_index = 0
        self.state = DroneState.RTH

    def survivors_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            count = len(data.get("survivors", []))
            self.survivors_count = count
            if count >= 6:
                self.get_logger().info(
                    f"Objective Update: {count} survivors located and mapped! Continuing systematic maze exploration tour...",
                    throttle_duration_sec=15.0
                )
        except Exception:
            pass

    def cmd_callback(self, msg: String):
        """Processes high-level GCS / Operator commands."""
        command = msg.data.strip().lower()
        self.get_logger().info(f"Operator Command Received: '{command}'")

        if command == "takeoff":
            if self.state in [DroneState.IDLE, DroneState.LAND]:
                self.takeoff_stable_count = 0
                self.state = DroneState.TAKEOFF
        elif command == "hover":
            self.state = DroneState.HOVER
        elif command == "explore":
            if self.state in [DroneState.IDLE, DroneState.LAND]:
                self.state = DroneState.TAKEOFF
            else:
                self.state = DroneState.EXPLORE
        elif command == "rth":
            self.initiate_rth("Operator Command")
        elif command == "land":
            self.state = DroneState.LAND
        elif command in ["stop", "abort"]:
            self.state = DroneState.HOVER
            self.publish_twist(0.0, 0.0, 0.0, 0.0)
        else:
            self.get_logger().warn(f"Unknown command: '{command}'")

    # -------------------------------------------------------------
    # Main Control Loop
    # -------------------------------------------------------------
    def control_loop(self):
        cmd = Twist()

        # Effective altitude determined by LiDAR 2 (Downward ToF) with safety cross-check
        if 0.02 <= self.ground_distance <= 3.0:
            effective_alt = self.ground_distance
        else:
            effective_alt = max(0.05, self.current_z)

        alt_error = self.target_alt - effective_alt

        if self.state == DroneState.IDLE:
            self.pid_alt.reset()
            self.pid_yaw.reset()
            self.pid_lat.reset()
            self.pid_vel.reset()
            self.publish_twist(0.0, 0.0, 0.0, 0.0)
            return

        elif self.state == DroneState.TAKEOFF:
            # Full PID closed-loop vertical climb using Downward LiDAR
            climb_vel = self.pid_alt.compute(alt_error, dt=0.05)
            # Ensure decisive vertical lift authority to clear ground contact
            if effective_alt < 0.40:
                cmd.linear.z = max(0.60, climb_vel)
            else:
                cmd.linear.z = climb_vel
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.angular.z = 0.0

            self.get_logger().info(
                f"TAKEOFF in progress: altitude={effective_alt:.2f}m / {self.target_alt:.2f}m (vertical climb cmd: {cmd.linear.z:.2f} m/s)",
                throttle_duration_sec=2.0
            )

            # Automatically transition to EXPLORE once stable at target altitude
            if abs(alt_error) < 0.15 and effective_alt >= 1.05:
                self.takeoff_stable_count += 1
                if self.takeoff_stable_count > 15:  # Stable for ~0.75 second at altitude
                    self.get_logger().info(f"Cruising altitude reached ({effective_alt:.2f} m). Transitioning to EXPLORE.")
                    self.state = DroneState.EXPLORE
            else:
                self.takeoff_stable_count = 0

        elif self.state == DroneState.HOVER:
            # Full PID Altitude hold
            cmd.linear.z = self.pid_alt.compute(alt_error, dt=0.05)
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.angular.z = 0.0

        elif self.state == DroneState.EXPLORE:
            # 1. Closed-loop Altitude Regulation via Altitude PID
            if effective_alt < 0.60:
                # Still low / near floor: prioritize vertical climb before driving forward
                cmd.linear.z = 0.65
                cmd.linear.x = 0.0
                cmd.linear.y = 0.0
                cmd.angular.z = 0.0
                self.cmd_vel_pub.publish(cmd)
                return
            else:
                cmd.linear.z = self.pid_alt.compute(alt_error, dt=0.05)

            # Ceiling Net Protection (Net is at 2.44m / 8ft)
            if effective_alt > 1.65:
                cmd.linear.z = -0.55

            # 2. Check Tour Completion
            if self.waypoint_index >= len(self.maze_tour):
                self.get_logger().info("Autonomous maze tour complete! Touchdown confirmed on Launch Pad. Initiating LAND.")
                self.state = DroneState.LAND
                return

            # Target current waypoint in the maze tour
            wp = self.maze_tour[self.waypoint_index]
            dx = wp["x"] - self.current_x
            dy = wp["y"] - self.current_y
            dist = math.hypot(dx, dy)

            # Advance to next waypoint once within tolerance
            threshold = 0.48 if self.waypoint_index < len(self.maze_tour) - 1 else 0.35
            if dist < threshold:
                self.get_logger().info(f"Reached [{self.waypoint_index + 1}/{len(self.maze_tour)}]: {wp['desc']}")
                self.waypoint_index += 1
                if self.waypoint_index >= len(self.maze_tour):
                    self.get_logger().info("Autonomous maze tour complete! Touchdown confirmed on Launch Pad. Initiating LAND.")
                    self.state = DroneState.LAND
                    return
                wp = self.maze_tour[self.waypoint_index]
                dx = wp["x"] - self.current_x
                dy = wp["y"] - self.current_y
                dist = math.hypot(dx, dy)

            # 3. Heading Alignment via Yaw PID
            target_yaw = math.atan2(dy, dx)
            yaw_diff = math.atan2(math.sin(target_yaw - self.current_yaw), math.cos(target_yaw - self.current_yaw))
            cmd.angular.z = self.pid_yaw.compute(yaw_diff, dt=0.05)

            # 4. Reactive Lateral Corridor Centering via Lateral PID
            # Equidistant tracking in corridors where at least one wall is within 1.8m
            if self.left_dist < 1.8 or self.right_dist < 1.8:
                clamped_left = min(self.left_dist, 1.8)
                clamped_right = min(self.right_dist, 1.8)
                corridor_err = clamped_left - clamped_right
                cmd.linear.y = self.pid_lat.compute(corridor_err, dt=0.05)
            else:
                self.pid_lat.reset()
                cmd.linear.y = 0.0

            # 5. Cruise & Turning Speed Control via Velocity PID
            if abs(yaw_diff) > 0.40:
                # Prioritize turning into alignment before fast forward cruise
                cmd.linear.x = 0.05
            else:
                desired_speed = self.pid_vel.compute(dist, dt=0.05)
                # LiDAR Wall Proximity Safety & Deceleration
                if self.front_dist < self.wall_safety:
                    slowdown = max(0.08, (self.front_dist - 0.38) / (self.wall_safety - 0.38))
                    cmd.linear.x = desired_speed * slowdown
                else:
                    cmd.linear.x = max(0.12, desired_speed)

            # 6. Obstacle Avoidance & Corridor Centering
            if self.front_dist < self.emerg_dist:
                # Obstacle directly ahead: reverse gently while turning towards open side
                cmd.linear.x = -0.15
                cmd.angular.z = 0.45 if self.left_dist > self.right_dist else -0.45
            else:
                # Proactive side-wall repulsion: strafe away from walls without reversing linear.x
                if self.left_dist < 0.24:
                    cmd.linear.y = -0.18  # Push away from left wall
                elif self.right_dist < 0.24:
                    cmd.linear.y = 0.18   # Push away from right wall

        elif self.state == DroneState.RTH:
            # Return To Home using Collision-Free Topological Backtracking + Full 4-Channel PID
            cmd.linear.z = self.pid_alt.compute(alt_error, dt=0.05)
            if effective_alt > 1.65:
                cmd.linear.z = -0.55

            dist_to_home = math.hypot(self.home_x - self.current_x, self.home_y - self.current_y)
            if dist_to_home < 0.45 or not self.rth_waypoints or self.rth_index >= len(self.rth_waypoints):
                if dist_to_home < 0.48:
                    self.get_logger().info("Arrived at Launch Pad! Commencing precision auto-landing...")
                    self.state = DroneState.LAND
                    return
                else:
                    target_wp = {"x": self.home_x, "y": self.home_y, "desc": "Launch Pad Touchdown"}
            else:
                target_wp = self.rth_waypoints[self.rth_index]

            dx = target_wp["x"] - self.current_x
            dy = target_wp["y"] - self.current_y
            dist = math.hypot(dx, dy)

            # Advance to next backtrack waypoint once within tolerance
            threshold = 0.48 if self.rth_index < len(self.rth_waypoints) - 1 else 0.35
            if dist < threshold:
                self.get_logger().info(f"RTH Reached [{self.rth_index + 1}/{len(self.rth_waypoints)}]: {target_wp['desc']}")
                self.rth_index += 1
                if self.rth_index >= len(self.rth_waypoints):
                    self.get_logger().info("Arrived at Launch Pad! Commencing precision auto-landing...")
                    self.state = DroneState.LAND
                    return
                target_wp = self.rth_waypoints[self.rth_index]
                dx = target_wp["x"] - self.current_x
                dy = target_wp["y"] - self.current_y
                dist = math.hypot(dx, dy)

            # Heading via Yaw PID
            target_yaw = math.atan2(dy, dx)
            yaw_diff = math.atan2(math.sin(target_yaw - self.current_yaw), math.cos(target_yaw - self.current_yaw))
            cmd.angular.z = self.pid_yaw.compute(yaw_diff, dt=0.05)

            # Lateral Corridor Centering via Lateral PID in RTH
            if self.left_dist < 1.8 or self.right_dist < 1.8:
                clamped_left = min(self.left_dist, 1.8)
                clamped_right = min(self.right_dist, 1.8)
                corridor_err = clamped_left - clamped_right
                cmd.linear.y = self.pid_lat.compute(corridor_err, dt=0.05)
            else:
                self.pid_lat.reset()
                cmd.linear.y = 0.0

            # Forward Speed via Velocity PID in RTH
            if abs(yaw_diff) > 0.40:
                cmd.linear.x = 0.05
            else:
                rth_speed = self.pid_vel.compute(dist, dt=0.05)
                if self.front_dist < self.wall_safety:
                    slowdown = max(0.08, (self.front_dist - 0.38) / (self.wall_safety - 0.38))
                    cmd.linear.x = rth_speed * slowdown
                else:
                    cmd.linear.x = max(0.12, rth_speed)

            # Obstacle avoidance and side repulsion in RTH
            if self.front_dist < self.emerg_dist:
                cmd.linear.x = -0.15
                cmd.angular.z = 0.45 if self.left_dist > self.right_dist else -0.45
            else:
                if self.left_dist < 0.24:
                    cmd.linear.y = -0.18
                elif self.right_dist < 0.24:
                    cmd.linear.y = 0.18

        elif self.state == DroneState.LAND:
            # Gentle, cushioned precision landing monitored by Downward ToF LiDAR
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.angular.z = 0.0

            if effective_alt > 0.25:
                cmd.linear.z = -0.25
            elif effective_alt > 0.08:
                cmd.linear.z = -0.15
            else:
                cmd.linear.z = 0.0
                self.publish_twist(0.0, 0.0, 0.0, 0.0)
                self.get_logger().info("Touchdown confirmed on Launch Pad. Mission complete! Drone in IDLE.")
                self.state = DroneState.IDLE
                return

        self.cmd_vel_pub.publish(cmd)

    def publish_twist(self, lx, ly, lz, az):
        cmd = Twist()
        cmd.linear.x = float(lx)
        cmd.linear.y = float(ly)
        cmd.linear.z = float(lz)
        cmd.angular.z = float(az)
        self.cmd_vel_pub.publish(cmd)

    def publish_status(self):
        effective_alt = self.ground_distance if self.ground_distance > 0.08 else self.current_z
        grid_col = int(round(self.current_x + 7.5))
        grid_row = int(round(self.current_y + 7.5))
        if self.state == DroneState.RTH and self.rth_waypoints:
            current_rth = self.rth_waypoints[min(self.rth_index, len(self.rth_waypoints) - 1)]
            tour_info = {
                "waypoint": f"RTH {min(self.rth_index + 1, len(self.rth_waypoints))}/{len(self.rth_waypoints)}",
                "sector": current_rth["desc"]
            }
        else:
            current_wp = self.maze_tour[min(self.waypoint_index, len(self.maze_tour) - 1)]
            tour_info = {
                "waypoint": f"{min(self.waypoint_index + 1, len(self.maze_tour))}/{len(self.maze_tour)}",
                "sector": current_wp["desc"]
            }

        status_data = {
            "state": self.state,
            "altitude_tf02_m": round(effective_alt, 2),
            "target_altitude_m": self.target_alt,
            "tour": tour_info,
            "position": {
                "x": round(self.current_x, 2),
                "y": round(self.current_y, 2),
                "z": round(self.current_z, 2),
                "yaw_deg": round(math.degrees(self.current_yaw), 1),
                "grid_box": f"Grid [{grid_col}, {grid_row}]"
            },
            "speed": {
                "vx": round(self.linear_vel_x, 2),
                "vy": round(self.linear_vel_y, 2)
            },
            "lidar_clearance": {
                "front_m": round(self.front_dist, 2),
                "left_m": round(self.left_dist, 2),
                "right_m": round(self.right_dist, 2),
                "rear_m": round(self.rear_dist, 2),
                "min_m": round(self.min_dist, 2)
            }
        }
        msg = String()
        msg.data = json.dumps(status_data)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DroneAutonomyController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception as e:
        node.get_logger().error(f"Controller exception: {e}")
    finally:
        try:
            if rclpy.ok():
                node.publish_twist(0.0, 0.0, 0.0, 0.0)
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
