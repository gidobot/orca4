#!/usr/bin/env python3
"""
Post-connect MAVROS configuration.

1. Set MAVROS stream rate for RAW_SENSORS (includes SCALED_PRESSURE / static_pressure).
   MAVROS sends REQUEST_DATA_STREAM on connect with a low default rate (~2 Hz).

2. Override COMPASS_EXTERNAL to 0 (BlueOS restores 1 on restart; SITL needs 0 for
   simulated compass from Gazebo). Uses force_set to bypass MAVROS param cache.

Usage:
  ros2 run orca_bringup configure_mavros.py [--rate RATE] [--no-compass-override] [--timeout TIMEOUT]
"""

import argparse
import sys
import time


def main():
    parser = argparse.ArgumentParser(
        description="Post-connect MAVROS config: stream rate + COMPASS_EXTERNAL override"
    )
    parser.add_argument(
        "--rate",
        type=int,
        default=20,
        help="RAW_SENSORS stream rate in Hz (default: 20). Use 0 to skip.",
    )
    parser.add_argument(
        "--no-compass-override",
        action="store_true",
        help="Skip setting COMPASS_EXTERNAL 0 (e.g. for real hardware)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=30.0,
        help="Seconds to wait for services (default: 30)",
    )
    args = parser.parse_args()
    if args.rate <= 0 and args.no_compass_override:
        sys.exit(0)

    import rclpy
    from rclpy.node import Node
    from mavros_msgs.srv import StreamRate
    from mavros_msgs.srv import ParamSetV2
    from rcl_interfaces.msg import ParameterValue, ParameterType

    rclpy.init()
    node = Node("configure_mavros")

    exit_code = 0

    # 1. Set stream rate
    if args.rate > 0:
        client = node.create_client(StreamRate, "/mavros/set_stream_rate")
        node.get_logger().info(f"Waiting for /mavros/set_stream_rate (timeout={args.timeout}s)...")
        if client.wait_for_service(timeout_sec=args.timeout):
            req = StreamRate.Request()
            req.stream_id = 1  # STREAM_RAW_SENSORS
            req.message_rate = args.rate
            req.on_off = True
            future = client.call_async(req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
            if future.done() and future.result() is not None:
                node.get_logger().info(f"Set RAW_SENSORS stream rate to {args.rate} Hz")
            else:
                node.get_logger().error("Stream rate service call failed or timed out")
                exit_code = 1
        else:
            node.get_logger().error("/mavros/set_stream_rate not available")
            exit_code = 1

    # 2. Override COMPASS_EXTERNAL to 0. Give MAVROS time to sync params, use force_set.
    if not args.no_compass_override:
        time.sleep(5)  # Allow MAVROS param cache to populate
        client = node.create_client(ParamSetV2, "/mavros/param/set")
        if client.wait_for_service(timeout_sec=args.timeout):
            pv = ParameterValue()
            pv.type = ParameterType.PARAMETER_INTEGER
            pv.integer_value = 0
            req = ParamSetV2.Request()
            req.force_set = True  # Bypass MAVROS cache, send directly to FCU
            req.param_id = "COMPASS_EXTERNAL"
            req.value = pv
            future = client.call_async(req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
            if future.done() and future.result() is not None and future.result().success:
                node.get_logger().info("Set COMPASS_EXTERNAL = 0")
            else:
                node.get_logger().warn(
                    "Could not set COMPASS_EXTERNAL=0 via MAVROS. "
                    "Set manually in BlueOS after each ArduPilot restart."
                )
        else:
            node.get_logger().warn("/mavros/param/set not available")

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
