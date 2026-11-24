#!/usr/bin/env python3
"""
NIDAR AirMouse - Ground Control Station (GCS) & Mission Planner Server
-----------------------------------------------------------------------
Full compliance with Problem Statement Section 5:
- Real-time 2D SLAM Map Canvas with live grid, drone trajectory & survivor pins
- Live Drone Camera Stream with survivor detection HUD
- Dual-LiDAR Telemetry (Altitude via Downward ToF, Clearance via 360 Horizontal)
- Survivor Localisation Log & Grid Box Coordinator
- Interactive Mission Control Dispatcher (Takeoff, Explore, RTH, Land, Abort)
Runs on http://localhost:8080
"""

import os
import io
import time
import json
import math
import threading
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_msgs.msg import String
from geometry_msgs.msg import Twist


# 36 Accurate Architectural Arena Wall Panels extracted from 3D Simulation SDF
ARENA_WALLS = [
    # Outer perimeter walls (-7.5 to +7.5)
    (-7.5, 0.0, 0.1, 15.0),
    (7.5, 0.0, 0.1, 15.0),
    (0.0, 7.5, 15.0, 0.1),
    (-6.3, -7.5, 2.4, 0.1),
    (1.8, -7.5, 11.4, 0.1),
    # Interior room partitions & corridor barriers
    (-4.0, -6.0, 0.08, 3.0),
    (-4.0, -3.75, 0.08, 1.5),
    (-4.0, 2.25, 0.08, 1.5),
    (-4.0, 5.85, 0.08, 3.3),
    (-6.35, -4.5, 2.3, 0.08),
    (-6.35, -1.5, 2.3, 0.08),
    (-6.35, 1.5, 2.3, 0.08),
    (-3.0, -5.0, 2.0, 0.08),
    (-1.5, 5.75, 0.08, 3.5),
    (-0.85, 4.0, 1.3, 0.08),
    (-0.5, -6.65, 0.08, 1.7),
    (-0.5, -3.55, 0.08, 2.1),
    (-0.5, 2.0, 2.0, 0.08),
    (0.25, -5.0, 1.5, 0.08),
    (0.5, -0.25, 0.08, 4.5),
    (1.75, 4.0, 1.5, 0.08),
    (2.0, -2.5, 3.0, 0.08),
    (2.5, 5.75, 0.08, 3.5),
    (2.85, -5.0, 1.3, 0.08),
    (3.5, -5.75, 0.08, 3.5),
    (3.5, -2.15, 0.08, 1.3),
    (3.5, -0.5, 0.08, 2.0),
    (3.5, 2.6, 0.08, 1.8),
    (4.0, 5.5, 0.08, 4.0),
    (4.15, -4.5, 1.3, 0.08),
    (4.65, 1.0, 2.3, 0.08),
    (5.5, -1.5, 4.0, 0.08),
    (5.5, 0.4, 0.08, 1.2),
    (6.35, 3.5, 2.3, 0.08),
    (6.75, -4.5, 1.5, 0.08),
    (7.25, 1.0, 0.5, 0.08)
]


def generate_2d_grid_map(state: dict) -> bytes:
    """Renders the comprehensive 2D LiDAR 1m x 1m grid map with survivor markings.
    Guaranteed to always produce a crisp, tactical view and NEVER a black screen."""
    w_img = 680
    h_img = 680
    margin = 40
    arena_px = 600

    def w2p(x, y):
        px = int(margin + (x + 7.5) / 15.0 * arena_px)
        py = int(margin + (7.5 - y) / 15.0 * arena_px)
        return px, py

    canvas = np.full((h_img, w_img, 3), (18, 22, 30), dtype=np.uint8)

    # 1. Arena floor
    p_tl = w2p(-7.5, 7.5)
    p_br = w2p(7.5, -7.5)
    cv2.rectangle(canvas, p_tl, p_br, (232, 236, 242), -1)

    # 2. 1m x 1m Grid lines & Coordinate Indices (Columns 0..14, Rows 0..14)
    for i in range(16):
        gx = -7.5 + i * 1.0
        p1 = w2p(gx, 7.5)
        p2 = w2p(gx, -7.5)
        cv2.line(canvas, p1, p2, (205, 215, 225), 1)
        if i < 15:
            # Column indices on top and bottom margins
            cv2.putText(canvas, str(i), (margin + i * 40 + 13, 26),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (100, 140, 180), 1)
            cv2.putText(canvas, str(i), (margin + i * 40 + 13, h_img - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (100, 140, 180), 1)

    for j in range(16):
        gy = -7.5 + j * 1.0
        p1 = w2p(-7.5, gy)
        p2 = w2p(7.5, gy)
        cv2.line(canvas, p1, p2, (205, 215, 225), 1)
        if j < 15:
            # Row indices on left and right margins
            cv2.putText(canvas, str(j), (14, margin + (14 - j) * 40 + 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (100, 140, 180), 1)
            cv2.putText(canvas, str(j), (w_img - 26, margin + (14 - j) * 40 + 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (100, 140, 180), 1)

    # 3. Live 2D LiDAR SLAM Occupancy Overlay (if active from slam_toolbox)
    slam = state.get("slam_occupancy")
    if slam is not None:
        try:
            sw, sh, sres, sox, soy, sdata = slam
            occ_ys, occ_xs = np.where(sdata > 50)
            if len(occ_xs) > 0:
                wx = sox + occ_xs * sres
                wy = soy + occ_ys * sres
                pxs = np.clip(margin + ((wx + 7.5) / 15.0 * arena_px).astype(np.int32), 0, w_img - 1)
                pys = np.clip(margin + ((7.5 - wy) / 15.0 * arena_px).astype(np.int32), 0, h_img - 1)
                canvas[pys, pxs] = (15, 20, 30)
        except Exception:
            pass

    # 4. Arena Architectural Walls
    for wx, wy, sx, sy in ARENA_WALLS:
        x1, y1 = w2p(wx - sx/2, wy + sy/2)
        x2, y2 = w2p(wx + sx/2, wy - sy/2)
        rx1, rx2 = min(x1, x2), max(x1, x2)
        ry1, ry2 = min(y1, y2), max(y1, y2)
        if rx2 - rx1 < 2: rx2 = rx1 + 2
        if ry2 - ry1 < 2: ry2 = ry1 + 2
        cv2.rectangle(canvas, (rx1, ry1), (rx2, ry2), (40, 50, 65), -1)

    # 5. Designated Single Takeoff & Landing Pad at (-5.8, -6.0)
    pad_p1 = w2p(-6.105, -5.695)
    pad_p2 = w2p(-5.495, -6.305)
    cv2.rectangle(canvas, pad_p1, pad_p2, (0, 200, 60), -1)
    cv2.rectangle(canvas, pad_p1, pad_p2, (0, 120, 30), 2)
    cv2.putText(canvas, "PAD", (pad_p1[0] - 2, max(35, pad_p1[1] - 4)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 180, 50), 1)

    # 6. MARKED SURVIVORS ON 1m x 1m GRID
    survivors = state.get("survivors", [])
    for s in survivors:
        sx = float(s.get("x", 0.0))
        sy = float(s.get("y", 0.0))
        sid = s.get("id", 1)
        col = max(0, min(14, int(math.floor(sx + 7.5))))
        row = max(0, min(14, int(math.floor(sy + 7.5))))
        xmin = -7.5 + col
        ymin = -7.5 + row

        c_tl = w2p(xmin, ymin + 1.0)
        c_br = w2p(xmin + 1.0, ymin)
        px1, py1 = c_tl
        px2, py2 = c_br

        # 1. High-visibility filled 1x1m cell (glowing amber)
        roi = canvas[py1:py2, px1:px2]
        if roi.size > 0:
            amber = np.full_like(roi, (0, 215, 255), dtype=np.uint8)
            cv2.addWeighted(roi, 0.50, amber, 0.50, 0, roi)
            canvas[py1:py2, px1:px2] = roi

        # 2. Bold rectangular border for the 1x1m cell
        cv2.rectangle(canvas, c_tl, c_br, (0, 140, 255), 2)

        # 3. Bold diagonal "X" cross mark across the 1x1m cell
        cv2.line(canvas, c_tl, c_br, (0, 30, 240), 2)
        cv2.line(canvas, (px1, py2), (px2, py1), (0, 30, 240), 2)

        # 4. Survivor exact location pin
        spx, spy = w2p(sx, sy)
        cv2.circle(canvas, (spx, spy), 5, (0, 30, 240), -1)
        cv2.circle(canvas, (spx, spy), 7, (255, 255, 255), 1)

        # 5. Badge Tag
        tag = f"S#{sid} [{col},{row}]"
        cv2.rectangle(canvas, (px1 - 2, max(6, py1 - 18)), (px1 + 72, py1 - 2), (18, 22, 30), -1)
        cv2.rectangle(canvas, (px1 - 2, max(6, py1 - 18)), (px1 + 72, py1 - 2), (0, 200, 255), 1)
        cv2.putText(canvas, tag, (px1, max(14, py1 - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1)

    # 7. Drone Flight Trajectory
    traj = state.get("trajectory", [])
    for pt in traj:
        if isinstance(pt, dict):
            tx, ty = pt.get("x", 0.0), pt.get("y", 0.0)
        elif isinstance(pt, (tuple, list)) and len(pt) >= 2:
            tx, ty = pt[0], pt[1]
        else:
            continue
        tpx, tpy = w2p(tx, ty)
        if 0 <= tpx < w_img and 0 <= tpy < h_img:
            cv2.circle(canvas, (tpx, tpy), 1, (255, 120, 0), -1)

    # 8. Drone Live Position & Heading
    cur_pos = state.get("status", {}).get("position", {})
    dx = cur_pos.get("x", -5.8)
    dy = cur_pos.get("y", -6.0)
    dyaw = math.radians(cur_pos.get("yaw_deg", 0.0))
    dpx, dpy = w2p(dx, dy)
    cv2.circle(canvas, (dpx, dpy), 8, (0, 200, 255), 2)
    cv2.circle(canvas, (dpx, dpy), 5, (0, 240, 255), -1)
    head_x = int(dpx + 14 * math.cos(dyaw))
    head_y = int(dpy - 14 * math.sin(dyaw))
    cv2.line(canvas, (dpx, dpy), (head_x, head_y), (0, 0, 255), 2)

    # 9. Tactical HUD Overlay
    cv2.putText(canvas, "2D LIDAR 1x1m GRID MAP", (margin + 4, 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 200), 1)
    sur_txt = f"SURVIVORS MARKED: {len(survivors)}/6"
    cv2.putText(canvas, sur_txt, (margin + arena_px - 170, 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 100), 1)

    _, jpeg = cv2.imencode(".jpg", canvas, [cv2.IMWRITE_JPEG_QUALITY, 90])
    return jpeg.tobytes()


# Global state shared between ROS 2 subscriber callbacks and HTTP Server
GCS_STATE = {
    "status": {
        "state": "STANDBY",
        "altitude_tf02_m": 0.05,
        "target_altitude_m": 1.22,
        "position": {"x": -5.8, "y": -6.0, "z": 0.05, "yaw_deg": 0.0, "grid_box": "Grid [1, 1]"},
        "speed": {"vx": 0.0, "vy": 0.0},
        "lidar_clearance": {"front_m": 10.0, "left_m": 5.0, "right_m": 5.0, "rear_m": 5.0, "min_m": 5.0}
    },
    "survivors": [],
    "trajectory": [{"x": -5.8, "y": -6.0}],
    "map_metadata": {"width": 0, "height": 0, "resolution": 0.05, "origin_x": 0.0, "origin_y": 0.0},
    "slam_occupancy": None,
    "latest_camera_jpeg": None,
    "latest_map_jpeg": None,
    "mission_start_time": None
}

# Pre-render initial map so it is NEVER None or black from startup
GCS_STATE["latest_map_jpeg"] = generate_2d_grid_map(GCS_STATE)

ROS_NODE_INSTANCE = None


class GCSRequestHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        # Suppress routine HTTP request logging in terminal
        pass

    def do_GET(self):
        if self.path == "/" or self.path == "/index.html":
            self.serve_dashboard()
        elif self.path == "/api/telemetry":
            self.serve_telemetry()
        elif self.path == "/api/camera_stream":
            self.serve_camera_stream()
        elif self.path == "/api/map_stream":
            self.serve_map_stream()
        elif self.path == "/api/map_image":
            self.serve_map_image()
        else:
            self.send_error(404, "Endpoint Not Found")

    def do_POST(self):
        if self.path == "/api/command":
            content_length = int(self.headers.get('Content-Length', 0))
            post_data = self.rfile.read(content_length)
            try:
                data = json.loads(post_data.decode('utf-8'))
                cmd = data.get('command', '').strip()
                if ROS_NODE_INSTANCE and cmd:
                    ROS_NODE_INSTANCE.send_drone_command(cmd)
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.end_headers()
                self.wfile.write(json.dumps({"status": "success", "command": cmd}).encode('utf-8'))
            except Exception as e:
                self.send_error(400, f"Invalid JSON: {e}")
        else:
            self.send_error(404, "Endpoint Not Found")

    def serve_telemetry(self):
        payload = {
            "status": GCS_STATE["status"],
            "survivors": GCS_STATE["survivors"],
            "trajectory": GCS_STATE["trajectory"][-100:],
            "uptime_sec": int(time.time() - GCS_STATE["mission_start_time"]) if GCS_STATE["mission_start_time"] else 0
        }
        body = json.dumps(payload).encode('utf-8')
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
        self.send_header('Connection', 'close')
        self.end_headers()
        self.wfile.write(body)

    def serve_map_image(self):
        jpeg = GCS_STATE.get("latest_map_jpeg")
        if jpeg is None:
            jpeg = generate_2d_grid_map(GCS_STATE)
            GCS_STATE["latest_map_jpeg"] = jpeg

        self.send_response(200)
        self.send_header('Content-Type', 'image/jpeg')
        self.send_header('Content-Length', str(len(jpeg)))
        self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
        self.send_header('Connection', 'close')
        self.end_headers()
        self.wfile.write(jpeg)

    def serve_camera_stream(self):
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
        self.send_header('Connection', 'close')
        self.end_headers()
        try:
            while True:
                jpeg = GCS_STATE["latest_camera_jpeg"]
                if jpeg is not None:
                    header = (
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n"
                        b"Content-Length: " + str(len(jpeg)).encode() + b"\r\n\r\n"
                    )
                    self.wfile.write(header)
                    self.wfile.write(jpeg)
                    self.wfile.write(b"\r\n")
                time.sleep(0.04)
        except Exception:
            pass

    def serve_map_stream(self):
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
        self.send_header('Connection', 'close')
        self.end_headers()
        try:
            while True:
                jpeg = GCS_STATE.get("latest_map_jpeg")
                if jpeg is None:
                    jpeg = generate_2d_grid_map(GCS_STATE)
                    GCS_STATE["latest_map_jpeg"] = jpeg
                header = (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    b"Content-Length: " + str(len(jpeg)).encode() + b"\r\n\r\n"
                )
                self.wfile.write(header)
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
                time.sleep(0.1)
        except Exception:
            pass

    def serve_dashboard(self):
        html = """<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>NIDAR AirMouse - Ground Control Station & Mission Planner</title>
  <link rel="preconnect" href="https://fonts.googleapis.com">
  <link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
  <link href="https://fonts.googleapis.com/css2?family=Inter:wght@300;400;500;600;700&family=JetBrains+Mono:wght@400;600&display=swap" rel="stylesheet">
  <style>
    :root {
      --bg-dark: #0a0d14;
      --card-bg: rgba(18, 24, 38, 0.85);
      --card-border: rgba(255, 255, 255, 0.08);
      --accent-cyan: #00f2fe;
      --accent-blue: #4facfe;
      --accent-green: #10b981;
      --accent-orange: #f59e0b;
      --accent-red: #ef4444;
      --text-main: #f3f4f6;
      --text-muted: #9ca3af;
      --font-ui: 'Inter', -apple-system, sans-serif;
      --font-mono: 'JetBrains Mono', monospace;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body {
      background-color: var(--bg-dark);
      color: var(--text-main);
      font-family: var(--font-ui);
      overflow-x: hidden;
      min-height: 100vh;
      background-image: 
        radial-gradient(circle at 15% 15%, rgba(0, 242, 254, 0.05) 0%, transparent 40%),
        radial-gradient(circle at 85% 85%, rgba(79, 172, 254, 0.04) 0%, transparent 40%);
    }
    header {
      background: rgba(13, 17, 28, 0.95);
      backdrop-filter: blur(12px);
      border-bottom: 1px solid var(--card-border);
      padding: 14px 28px;
      display: flex;
      justify-content: space-between;
      align-items: center;
      position: sticky;
      top: 0;
      z-index: 100;
    }
    .logo-group { display: flex; align-items: center; gap: 14px; }
    .badge-icon {
      width: 36px; height: 36px;
      background: linear-gradient(135deg, var(--accent-cyan), var(--accent-blue));
      border-radius: 8px;
      display: flex; align-items: center; justify-content: center;
      font-weight: 700; color: #000; font-size: 18px;
      box-shadow: 0 0 15px rgba(0, 242, 254, 0.4);
    }
    .header-titles h1 { font-size: 1.15rem; font-weight: 700; letter-spacing: 0.5px; }
    .header-titles p { font-size: 0.78rem; color: var(--text-muted); }
    .header-status { display: flex; align-items: center; gap: 20px; font-family: var(--font-mono); font-size: 0.85rem; }
    .state-pill {
      padding: 5px 12px;
      border-radius: 20px;
      font-weight: 600;
      text-transform: uppercase;
      letter-spacing: 1px;
      font-size: 0.75rem;
      background: rgba(16, 185, 129, 0.15);
      border: 1px solid var(--accent-green);
      color: var(--accent-green);
    }
    .container {
      max-width: 1600px;
      margin: 0 auto;
      padding: 24px;
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 24px;
    }
    .full-width { grid-column: 1 / -1; }
    .card {
      background: var(--card-bg);
      border: 1px solid var(--card-border);
      border-radius: 12px;
      padding: 18px 22px;
      backdrop-filter: blur(8px);
      box-shadow: 0 10px 30px rgba(0,0,0,0.4);
      display: flex;
      flex-direction: column;
    }
    .card-title {
      font-size: 0.92rem;
      font-weight: 600;
      color: var(--accent-cyan);
      text-transform: uppercase;
      letter-spacing: 1px;
      margin-bottom: 14px;
      display: flex;
      justify-content: space-between;
      align-items: center;
    }
    .telemetry-grid {
      display: grid;
      grid-template-columns: repeat(4, 1fr);
      gap: 12px;
      margin-bottom: 8px;
    }
    .metric-box {
      background: rgba(255, 255, 255, 0.03);
      border: 1px solid rgba(255, 255, 255, 0.05);
      border-radius: 8px;
      padding: 12px;
    }
    .metric-label { font-size: 0.72rem; color: var(--text-muted); text-transform: uppercase; letter-spacing: 0.5px; }
    .metric-val { font-size: 1.3rem; font-weight: 700; font-family: var(--font-mono); margin-top: 4px; }
    .metric-unit { font-size: 0.8rem; font-weight: 400; color: var(--text-muted); }
    .feed-container {
      width: 100%;
      height: 380px;
      background: #121620;
      border-radius: 8px;
      overflow: hidden;
      display: flex;
      align-items: center;
      justify-content: center;
      position: relative;
      border: 1px solid rgba(0, 242, 254, 0.15);
    }
    .feed-container img {
      width: 100%;
      height: 100%;
      object-fit: contain;
    }
    .feed-overlay {
      position: absolute;
      top: 10px;
      left: 10px;
      background: rgba(0,0,0,0.6);
      padding: 4px 8px;
      border-radius: 4px;
      font-family: var(--font-mono);
      font-size: 0.72rem;
      color: var(--accent-cyan);
    }
    .controls-bar {
      display: flex;
      gap: 12px;
      flex-wrap: wrap;
    }
    button.btn {
      padding: 10px 20px;
      border-radius: 6px;
      font-family: var(--font-ui);
      font-size: 0.82rem;
      font-weight: 600;
      cursor: pointer;
      transition: all 0.2s ease;
      display: flex;
      align-items: center;
      gap: 8px;
      border: 1px solid transparent;
    }
    .btn-cyan {
      background: linear-gradient(135deg, rgba(0, 242, 254, 0.2), rgba(79, 172, 254, 0.2));
      border-color: var(--accent-cyan);
      color: var(--accent-cyan);
    }
    .btn-cyan:hover {
      background: linear-gradient(135deg, var(--accent-cyan), var(--accent-blue));
      color: #000;
      box-shadow: 0 0 15px rgba(0, 242, 254, 0.4);
    }
    .btn-green {
      background: rgba(16, 185, 129, 0.2);
      border-color: var(--accent-green);
      color: var(--accent-green);
    }
    .btn-green:hover {
      background: var(--accent-green);
      color: #000;
      box-shadow: 0 0 15px rgba(16, 185, 129, 0.4);
    }
    .btn-orange {
      background: rgba(245, 158, 11, 0.2);
      border-color: var(--accent-orange);
      color: var(--accent-orange);
    }
    .btn-orange:hover {
      background: var(--accent-orange);
      color: #000;
      box-shadow: 0 0 15px rgba(245, 158, 11, 0.4);
    }
    .btn-red {
      background: rgba(239, 68, 68, 0.2);
      border-color: var(--accent-red);
      color: var(--accent-red);
    }
    .btn-red:hover {
      background: var(--accent-red);
      color: #fff;
      box-shadow: 0 0 15px rgba(239, 68, 68, 0.5);
    }
    table.survivor-table {
      width: 100%;
      border-collapse: collapse;
      font-size: 0.82rem;
      margin-top: 8px;
    }
    table.survivor-table th {
      text-align: left;
      padding: 8px 12px;
      border-bottom: 1px solid rgba(255, 255, 255, 0.1);
      color: var(--text-muted);
      font-weight: 500;
      text-transform: uppercase;
      font-size: 0.72rem;
    }
    table.survivor-table td {
      padding: 10px 12px;
      border-bottom: 1px solid rgba(255, 255, 255, 0.05);
      font-family: var(--font-mono);
    }
    .tag-room {
      background: rgba(0, 242, 254, 0.12);
      color: var(--accent-cyan);
      padding: 2px 8px;
      border-radius: 4px;
      font-weight: 500;
    }
    .tag-grid {
      background: rgba(245, 158, 11, 0.15);
      color: var(--accent-orange);
      padding: 2px 8px;
      border-radius: 4px;
      font-weight: 600;
    }
  </style>
</head>
<body>
  <header>
    <div class="logo-group">
      <div class="badge-icon">N</div>
      <div class="header-titles">
        <h1>NIDAR AirMouse | GCS Mission Planner</h1>
        <p>Autonomous Indoor GPS-Denied Search, 2D SLAM Mapping & Survivor Localisation</p>
      </div>
    </div>
    <div class="header-status">
      <div>ELAPSED: <span id="mission-clock" style="color: var(--accent-cyan);">00:00</span></div>
      <div class="state-pill" id="drone-state-pill">STANDBY</div>
    </div>
  </header>

  <div class="container">
    <!-- Telemetry & Sensor Overview -->
    <div class="card full-width">
      <div class="card-title">
        <span>Dual-LiDAR Flight Telemetry & Indoor Boundary Protection</span>
        <span style="font-size: 0.75rem; color: var(--text-muted); font-family: var(--font-mono);">GPS: DENIED (INDOOR ARENA)</span>
      </div>
      <div class="telemetry-grid">
        <div class="metric-box">
          <div class="metric-label">LiDAR 2 ToF Altitude</div>
          <div class="metric-val" id="alt-val" style="color: var(--accent-cyan);">0.05 <span class="metric-unit">m</span></div>
          <div style="font-size: 0.68rem; color: var(--text-muted); margin-top: 4px;">Limit: 2.44m Net | Cruising: 1.20m</div>
        </div>
        <div class="metric-box">
          <div class="metric-label">Drone Grid Position</div>
          <div class="metric-val" id="pos-val" style="color: var(--accent-green); font-size: 1.05rem;">[-5.8, -6.0]</div>
          <div style="font-size: 0.68rem; color: var(--text-muted); margin-top: 4px;" id="gridbox-val">Grid [2, 2] | Entry Pad</div>
        </div>
        <div class="metric-box">
          <div class="metric-label">LiDAR 1 Wall Clearances</div>
          <div class="metric-val" id="clearance-val" style="font-size: 1.0rem; color: var(--accent-orange);">F: 10.0m | L: 5.0m</div>
          <div style="font-size: 0.68rem; color: var(--text-muted); margin-top: 4px;" id="side-clearance">R: 5.0m | Rear: 5.0m</div>
        </div>
        <div class="metric-box">
          <div class="metric-label">Tagged Survivors</div>
          <div class="metric-val" id="survivor-count-val" style="color: #ff3366;">0 / 6</div>
          <div style="font-size: 0.68rem; color: var(--text-muted); margin-top: 4px;">Camera Localized & Map Tagged</div>
        </div>
      </div>
    </div>

    <!-- Live 2D SLAM Map Stream (Section 5 Requirement) -->
    <div class="card">
      <div class="card-title">
        <span>2D LiDAR 1m × 1m Grid Map & Survivor Marks</span>
        <span style="font-size: 0.72rem; color: var(--accent-green);">● LIVE 1x1m MARKING</span>
      </div>
      <div class="feed-container">
        <img id="map-img" src="/api/map_stream" alt="2D SLAM Map">
        <div class="feed-overlay">2D LiDAR 1m x 1m Grid Mapping & Survivor Marks</div>
      </div>
    </div>

    <!-- Live Drone Camera Feed (Section 1 & 5 Requirement) -->
    <div class="card">
      <div class="card-title">
        <span>Live Onboard Camera Stream & Localisation HUD</span>
        <span style="font-size: 0.72rem; color: var(--accent-cyan);">● 30 FPS GCS FEED</span>
      </div>
      <div class="feed-container">
        <img id="cam-img" src="/api/camera_stream" alt="Live Camera Feed">
        <div class="feed-overlay">Camera: 640x480 RGB | Active Tagging</div>
      </div>
    </div>

    <!-- Survivor Localisation Table (Section 6 Requirement) -->
    <div class="card full-width">
      <div class="card-title">
        <span>Survivor Detection, Grid Coordinates & Map Markers (Up to 6 Targets)</span>
        <span style="font-size: 0.72rem; color: var(--text-muted);">Tagged in Real-Time During Flight</span>
      </div>
      <table class="survivor-table">
        <thead>
          <tr>
            <th>ID</th>
            <th>Designation</th>
            <th>Arena Room / Sector</th>
            <th>Grid Coordinate (X, Y)</th>
            <th>Marked 1x1 Grid Cell</th>
            <th>Status</th>
          </tr>
        </thead>
        <tbody id="survivors-tbody">
          <tr><td colspan="6" style="text-align: center; color: var(--text-muted); padding: 18px;">No survivors detected yet. Exploring corridors...</td></tr>
        </tbody>
      </table>
    </div>

    <!-- Autonomous Mission Control Buttons -->
    <div class="card full-width">
      <div class="card-title">
        <span>Mission Planner & Emergency Failsafe Controls</span>
        <span style="font-size: 0.72rem; color: var(--text-muted);">No Manual Piloting | Autonomous Command Trigger Only</span>
      </div>
      <div class="controls-bar">
        <button class="btn btn-cyan" onclick="sendCmd('takeoff')">
          ▲ Takeoff (1.22m / 4ft Hold)
        </button>
        <button class="btn btn-green" onclick="sendCmd('explore')">
          ▶ Explore Arena Maze
        </button>
        <button class="btn btn-orange" onclick="sendCmd('hover')">
          ⏸ Hold / Hover
        </button>
        <button class="btn btn-cyan" onclick="sendCmd('rth')">
          ⤺ Return To Entry / Launch Pad (RTH)
        </button>
        <button class="btn btn-orange" onclick="sendCmd('land')">
          ▼ Precision Auto-Land
        </button>
        <button class="btn btn-red" onclick="sendCmd('abort')">
          ✕ EMERGENCY STOP
        </button>
      </div>
    </div>
  </div>

  <script>
    function sendCmd(command) {
      fetch('/api/command', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ command: command })
      })
      .then(res => res.json())
      .then(data => console.log('Command acknowledged:', data))
      .catch(err => console.error('Command failed:', err));
    }

    // Refresh telemetry and map periodically
    setInterval(() => {
      fetch('/api/telemetry')
      .then(r => r.json())
      .then(data => {
        const s = data.status;
        document.getElementById('drone-state-pill').innerText = s.state;
        document.getElementById('alt-val').innerHTML = `${s.altitude_tf02_m.toFixed(2)} <span class="metric-unit">m</span>`;
        document.getElementById('pos-val').innerText = `[${s.position.x.toFixed(1)}, ${s.position.y.toFixed(1)}]`;
        const tour = s.tour;
        const tourInfo = tour ? ` | ${tour.sector} (${tour.waypoint})` : '';
        document.getElementById('gridbox-val').innerText = `${s.position.grid_box}${tourInfo}`;
        
        const c = s.lidar_clearance;
        document.getElementById('clearance-val').innerText = `F: ${c.front_m.toFixed(1)}m | L: ${c.left_m.toFixed(1)}m`;
        document.getElementById('side-clearance').innerText = `R: ${c.right_m.toFixed(1)}m | Rear: ${c.rear_m.toFixed(1)}m | Min: ${c.min_m.toFixed(1)}m`;

        const sur = data.survivors;
        document.getElementById('survivor-count-val').innerText = `${sur.length} / 6`;

        // Update survivors table
        const tbody = document.getElementById('survivors-tbody');
        if (sur.length > 0) {
          tbody.innerHTML = sur.map(item => {
            const desig = item.name || item.label || `Survivor Target #${item.id}`;
            const gridLabel = item.grid_box || (item.grid_cell ? `Grid [${item.grid_cell.col}, ${item.grid_cell.row}] (1x1m)` : '1x1m Cell');
            return `
            <tr>
              <td><strong>#${item.id}</strong></td>
              <td>${desig}</td>
              <td><span class="tag-room">${item.room}</span></td>
              <td>[X: ${item.x.toFixed(1)}, Y: ${item.y.toFixed(1)}]</td>
              <td><span class="tag-grid">${gridLabel}</span></td>
              <td style="color: #00ff88; font-weight: 700;">✓ MARKED ON 1x1 GRID</td>
            </tr>
            `;
          }).join('');
        }

        // Mission timer
        const sec = data.uptime_sec;
        const m = Math.floor(sec / 60).toString().padStart(2, '0');
        const s_sec = (sec % 60).toString().padStart(2, '0');
        document.getElementById('mission-clock').innerText = `${m}:${s_sec}`;
      })
      .catch(() => {});

      // Ensure map stream auto-recovers if disconnected
      const mapImg = document.getElementById('map-img');
      if (mapImg && !mapImg.onerror) {
        mapImg.onerror = () => {
          setTimeout(() => { mapImg.src = '/api/map_stream?t=' + Date.now(); }, 1500);
        };
      }
    }, 1000);
  </script>
</body>
</html>
"""
        body = html.encode('utf-8')
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Connection', 'close')
        self.end_headers()
        self.wfile.write(body)


class MissionPlannerGCSNode(Node):
    def __init__(self):
        super().__init__('mission_planner_gcs')
        global ROS_NODE_INSTANCE
        ROS_NODE_INSTANCE = self
        self.bridge = CvBridge()

        GCS_STATE["mission_start_time"] = time.time()

        # Publishers
        self.cmd_pub = self.create_publisher(String, '/drone/cmd', 10)

        # QoS Profiles
        map_latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # Subscribers
        self.status_sub = self.create_subscription(String, '/drone/status', self.status_callback, 10)
        self.survivors_sub = self.create_subscription(String, '/drone/survivors', self.survivors_callback, 10)
        # Latched /map subscription for slam_toolbox
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_latched_qos)
        # Volatile /map subscription fallback
        self.map_vol_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        # Dedicated 1m x 1m survivor grid map
        self.survivor_grid_sub = self.create_subscription(OccupancyGrid, '/drone/survivor_grid_map', self.survivor_grid_callback, map_latched_qos)

        self.cam_sub = self.create_subscription(Image, '/drone/camera/annotated', self.annotated_camera_callback, 10)
        self.raw_cam_sub = self.create_subscription(Image, '/camera/image_raw', self.raw_camera_callback, 10)

        # Initial map render guarantees the 2D map is NEVER a black screen
        self.render_composite_map()

        # Fast 4 Hz refresh timer to smoothly render drone trajectory and marked survivors
        self.map_timer = self.create_timer(0.25, self.render_composite_map)

        self.get_logger().info("=" * 60)
        self.get_logger().info("NIDAR Ground Control Station & Mission Planner Server Online")
        self.get_logger().info("2D 1m x 1m Grid Map & Survivor Marking System Active")
        self.get_logger().info("Access Dashboard at: http://localhost:8080")
        self.get_logger().info("=" * 60)

    def render_composite_map(self):
        try:
            jpeg_bytes = generate_2d_grid_map(GCS_STATE)
            GCS_STATE["latest_map_jpeg"] = jpeg_bytes
        except Exception:
            pass

    def send_drone_command(self, cmd_text: str):
        msg = String()
        msg.data = cmd_text
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"GCS Dispatched Autonomous Command: '{cmd_text}'")

    def status_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            GCS_STATE["status"] = data
            x = data.get("position", {}).get("x", -5.8)
            y = data.get("position", {}).get("y", -6.0)
            GCS_STATE["trajectory"].append({"x": x, "y": y})
            if len(GCS_STATE["trajectory"]) > 2000:
                GCS_STATE["trajectory"].pop(0)
            self.render_composite_map()
        except Exception:
            pass

    def survivors_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            GCS_STATE["survivors"] = data.get("survivors", [])
            self.render_composite_map()
        except Exception:
            pass

    def survivor_grid_callback(self, msg: OccupancyGrid):
        # Triggered whenever new survivor cells are flagged in occupancy grid
        self.render_composite_map()

    def raw_camera_callback(self, msg: Image):
        # Fallback if annotated camera is not active yet
        if GCS_STATE["latest_camera_jpeg"] is None:
            try:
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                _, jpeg = cv2.imencode('.jpg', cv_img, [cv2.IMWRITE_JPEG_QUALITY, 80])
                GCS_STATE["latest_camera_jpeg"] = jpeg.tobytes()
            except Exception:
                pass

    def annotated_camera_callback(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            _, jpeg = cv2.imencode('.jpg', cv_img, [cv2.IMWRITE_JPEG_QUALITY, 85])
            GCS_STATE["latest_camera_jpeg"] = jpeg.tobytes()
        except Exception:
            pass

    def map_callback(self, msg: OccupancyGrid):
        try:
            w = msg.info.width
            h = msg.info.height
            res = msg.info.resolution
            ox = msg.info.origin.position.x
            oy = msg.info.origin.position.y

            if w == 0 or h == 0:
                return

            raw_data = np.array(msg.data, dtype=np.int8).reshape((h, w))
            GCS_STATE["slam_occupancy"] = (w, h, res, ox, oy, raw_data)
            GCS_STATE["map_metadata"] = {
                "width": w, "height": h, "resolution": res, "origin_x": ox, "origin_y": oy
            }
            self.render_composite_map()
        except Exception:
            pass


class ReusableThreadingHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = True
    daemon_threads = True


def start_http_server(preferred_port=8080):
    ports_to_try = [preferred_port, 8081, 8082, 8083, 8085, 5000]
    # Remove duplicates while preserving order
    candidate_ports = []
    for p in ports_to_try:
        if p not in candidate_ports:
            candidate_ports.append(p)

    httpd = None
    active_port = None
    for port in candidate_ports:
        try:
            server_address = ('0.0.0.0', port)
            httpd = ReusableThreadingHTTPServer(server_address, GCSRequestHandler)
            active_port = port
            GCS_STATE["server_port"] = active_port
            print("=" * 60, flush=True)
            print(f"[GCS Server] ONLINE on port {active_port}!", flush=True)
            print(f"  -> Local URL : http://localhost:{active_port}", flush=True)
            print(f"  -> Network   : http://0.0.0.0:{active_port}", flush=True)
            print("=" * 60, flush=True)
            break
        except OSError as e:
            print(f"[GCS Server] Port {port} unavailable ({e}), trying fallback...", flush=True)
            continue

    if httpd:
        httpd.serve_forever()
    else:
        print("[GCS Server] ERROR: Could not bind to any available port!", flush=True)


def main(args=None):
    rclpy.init(args=args)

    # Start HTTP GCS server in background daemon thread
    http_thread = threading.Thread(target=start_http_server, daemon=True)
    http_thread.start()

    node = MissionPlannerGCSNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception:
        pass
    finally:
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
