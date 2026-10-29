#!/usr/bin/env python3
"""
NIDAR AirMouse - Live Terminal Mission Monitor
------------------------------------------------
Displays real-time mission status, dual-LiDAR telemetry,
tagged survivors, and ASCII arena map directly inside your terminal.
"""

import time
import json
import urllib.request
import os
import sys

def clear_screen():
    sys.stdout.write("\033[H\033[J")
    sys.stdout.flush()

def draw_ascii_map(drone_x, drone_y, survivors):
    # Arena is 15m x 15m from X=[-7.5, 7.5], Y=[-7.5, 7.5]
    # Grid of 15 x 15 characters
    grid = [["." for _ in range(16)] for _ in range(16)]

    # Mark survivors
    for s in survivors:
        sx = int(round(s["x"] + 7.5))
        sy = 15 - int(round(s["y"] + 7.5))
        if 0 <= sx <= 15 and 0 <= sy <= 15:
            grid[sy][sx] = f"\033[91mS{s['id']}\033[0m"

    # Mark drone
    dx = int(round(drone_x + 7.5))
    dy = 15 - int(round(drone_y + 7.5))
    if 0 <= dx <= 15 and 0 <= dy <= 15:
        grid[dy][dx] = "\033[96m▲D\033[0m"

    lines = []
    lines.append("       +--------------------------------+")
    for r in range(16):
        row_str = " ".join([f"{c:>2}" for c in grid[r]])
        lines.append(f"       | {row_str} |")
    lines.append("       +--------------------------------+")
    lines.append("         Entry Pad: (-5.8, -6.0) | Scale: ~1m/cell")
    return "\n".join(lines)

def main():
    print("Connecting to NIDAR Mission Server...")
    while True:
        try:
            req = urllib.request.Request("http://localhost:8080/api/telemetry")
            with urllib.request.urlopen(req, timeout=1.5) as response:
                data = json.loads(response.read().decode())
            
            s = data["status"]
            pos = s["position"]
            lidar = s["lidar_clearance"]
            survivors = data["survivors"]
            sec = data["uptime_sec"]
            mins = sec // 60
            secs = sec % 60

            clear_screen()
            print("\033[1;36m" + "=" * 76 + "\033[0m")
            print(f"\033[1;37m   NIDAR AirMouse - DUAL-LIDAR AUTONOMOUS TERMINAL MONITOR\033[0m")
            print(f"   Mission Clock: \033[1;33m{mins:02d}:{secs:02d}\033[0m  |  Flight State: \033[1;32m{s['state']}\033[0m")
            print("\033[1;36m" + "=" * 76 + "\033[0m")

            # Telemetry section
            print("\033[1;34m[ DUAL-LIDAR FLIGHT TELEMETRY ]\033[0m")
            print(f"  * LiDAR 2 ToF Altitude : \033[1;32m{s['altitude_tf02_m']:.2f} m\033[0m (Target: {s['target_altitude_m']:.2f}m | Limit: 2.44m Net)")
            tour = s.get("tour", {})
            tour_str = f" | \033[1;36mTour [{tour.get('waypoint', '')}]: {tour.get('sector', '')}\033[0m" if tour else ""
            print(f"  * Arena Grid Sector    : \033[1;33m{pos['grid_box']}\033[0m{tour_str}")
            print(f"  * LiDAR 1 Clearances   : Front: \033[1;37m{lidar['front_m']:.2f}m\033[0m | Left: \033[1;37m{lidar['left_m']:.2f}m\033[0m | Right: \033[1;37m{lidar['right_m']:.2f}m\033[0m | Min: \033[1;31m{lidar['min_m']:.2f}m\033[0m")

            print("\n\033[1;34m[ 15m x 15m ARENA REAL-TIME TRACKER ]\033[0m")
            print(draw_ascii_map(pos["x"], pos["y"], survivors))

            print(f"\n\033[1;34m[ TAGGED SURVIVORS: {len(survivors)} / 6 DETECTED ]\033[0m")
            if len(survivors) == 0:
                print("  Exploring corridors... No survivors in camera FOV yet.")
            else:
                print(f"  {'ID':<5} {'Designation':<28} {'Room':<18} {'Grid Box':<12} {'Coords':<15}")
                print("  " + "-" * 74)
                for item in survivors:
                    print(f"  #{item['id']:<4} {item['name']:<28} {item['room']:<18} {item['grid_box']:<12} ({item['x']:.1f}, {item['y']:.1f})")

            print("\n\033[1;36m" + "=" * 76 + "\033[0m")
            print("  Commands: ./test_autonomous.sh [takeoff | explore | rth | land | stop]")
            print("  Press Ctrl+C to exit monitor.")

        except Exception as e:
            clear_screen()
            print("Waiting for simulation server at http://localhost:8080...")
            print(f"Status: {e}")

        time.sleep(0.5)

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\nExiting Terminal Monitor.")
