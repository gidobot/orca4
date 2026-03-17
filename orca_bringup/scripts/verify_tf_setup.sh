#!/bin/bash
# Verify TF setup for orca4. Run with sim + bringup running.
# Usage: ./verify_tf_setup.sh

echo "=== TF Tree ==="
ros2 run tf2_tools view_frames 2>/dev/null && echo "Generated frames.pdf" || echo "Run: ros2 run tf2_tools view_frames"

echo ""
echo "=== Key transforms (map -> base_link) ==="
timeout 2 ros2 run tf2_ros tf2_echo map base_link 2>&1 || echo "Could not get map->base_link"

echo ""
echo "=== TF publishers ==="
ros2 topic info /tf -v 2>/dev/null | grep -A1 "Publisher" || true
