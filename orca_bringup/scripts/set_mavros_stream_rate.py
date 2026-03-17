#!/usr/bin/env python3
"""
Set MAVROS stream rate for RAW_SENSORS (includes SCALED_PRESSURE / static_pressure).

MAVROS sends REQUEST_DATA_STREAM on connect with a low default rate (~2 Hz).
This script requests a higher rate so /mavros/imu/static_pressure publishes faster.

Usage:
  ros2 run orca_bringup set_mavros_stream_rate.py [--rate RATE] [--timeout TIMEOUT]
"""

import argparse
import sys


def main():
    parser = argparse.ArgumentParser(
        description="Set MAVROS RAW_SENSORS stream rate (SCALED_PRESSURE, etc.)"
    )
    parser.add_argument(
        "--rate",
        type=int,
        default=20,
        help="Stream rate in Hz (default: 20). Use 0 to skip.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=30.0,
        help="Seconds to wait for service (default: 30)",
    )
    args = parser.parse_args()
    if args.rate <= 0:
        sys.exit(0)

    import rclpy
    from rclpy.node import Node
    from mavros_msgs.srv import StreamRate

    rclpy.init()
    node = Node("set_mavros_stream_rate")

    service_name = "/mavros/set_stream_rate"
    client = node.create_client(StreamRate, service_name)
    node.get_logger().info(f"Waiting for {service_name} (timeout={args.timeout}s)...")
    if not client.wait_for_service(timeout_sec=args.timeout):
        node.get_logger().error(f"Service {service_name} not available")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)
    req = StreamRate.Request()
    req.stream_id = 1  # STREAM_RAW_SENSORS
    req.message_rate = args.rate
    req.on_off = True

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)

    if future.done() and future.result() is not None:
        node.get_logger().info(f"Set RAW_SENSORS stream rate to {args.rate} Hz")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)
    else:
        node.get_logger().error("Service call failed or timed out")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)


if __name__ == "__main__":
    main()
