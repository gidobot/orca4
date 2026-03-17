#!/usr/bin/env python3
"""
Set EKF origin for ArduPilot when using velocity-only mode (EK3_SRC1_POSXY=0).

ArduPilot 4.8 does not support MAV_CMD_SET_GLOBAL_ORIGIN; it requires the
SET_GPS_GLOBAL_ORIGIN MAVLink message. Run this script after MAVROS connects.

Usage:
  ros2 run orca_bringup set_ekf_origin.py [--lat LAT] [--lon LON] [--alt ALT]
  # Default connection matches MAVROS (udp:14550). For SITL direct: tcp:127.0.0.1:5760

Requires: pip install pymavlink
"""

import argparse
import sys


def main():
    parser = argparse.ArgumentParser(description="Set EKF origin for ArduPilot velocity-only mode")
    parser.add_argument("--lat", type=float, default=0.0, help="Latitude (degrees)")
    parser.add_argument("--lon", type=float, default=0.0, help="Longitude (degrees)")
    parser.add_argument("--alt", type=float, default=0.0, help="Altitude AMSL (m)")
    parser.add_argument(
        "--connection",
        default="udp:127.0.0.1:14550",
        help="MAVLink connection (default: udp:127.0.0.1:14550, matches MAVROS)",
    )
    args = parser.parse_args()

    try:
        from pymavlink import mavutil
    except ImportError:
        print("ERROR: pymavlink required. Install with: pip install pymavlink", file=sys.stderr)
        sys.exit(1)

    print(f"Connecting to {args.connection}...")
    conn = mavutil.mavlink_connection(args.connection)
    conn.wait_heartbeat()
    print("Heartbeat received")

    # SET_GPS_GLOBAL_ORIGIN: target_system, lat (degE7), lon (degE7), alt (mm)
    lat_e7 = int(args.lat * 1e7)
    lon_e7 = int(args.lon * 1e7)
    alt_mm = int(args.alt * 1000)

    # SET_GPS_GLOBAL_ORIGIN: target_system, lat (degE7), lon (degE7), alt (mm), time_usec
    conn.mav.set_gps_global_origin_send(
        0,  # target_system (0 = broadcast to all)
        lat_e7,
        lon_e7,
        alt_mm,
        0,  # time_usec
    )
    print(f"Sent SET_GPS_GLOBAL_ORIGIN: lat={args.lat}, lon={args.lon}, alt={args.alt}m")
    conn.close()
    print("Done")


if __name__ == "__main__":
    main()
