#!/bin/bash
# Debug script: verify Gazebo ↔ ArduPilot SITL connection for BlueOS
#
# The ArduPilotPlugin sends sensor data (pose, IMU, baro, mag) via UDP to
# ArduPilot SITL on port 9002. If BlueOS ArduPilot runs in Docker, network
# connectivity must be correct.
#
# Usage: Run while Gazebo and BlueOS ArduPilot are (or should be) running.

set -e

FDM_PORT=9002
MAVLINK_PORT=14551

echo "=== Gazebo ↔ ArduPilot SITL Connection Debug ==="
echo ""

# 1. Check if anything is listening on 9002 (Gazebo plugin or ArduPilot)
echo "1. Port $FDM_PORT (FDM/sensor data):"
if command -v ss &>/dev/null; then
    ss -ulnp 2>/dev/null | grep -E ":$FDM_PORT |$FDM_PORT " || echo "   (ss: no listener found)"
fi
if command -v netstat &>/dev/null; then
    netstat -ulnp 2>/dev/null | grep -E ":$FDM_PORT |$FDM_PORT " || echo "   (netstat: no listener found)"
fi
echo "   Expected: Gazebo's ArduPilotPlugin sends TO ArduPilot. ArduPilot SITL"
echo "   with -M JSON listens for incoming. Plugin uses fdm_addr:fdm_port_in."
echo ""

# 2. Check MAVLink port (ArduPilot → MAVROS)
echo "2. Port $MAVLINK_PORT (MAVLink, ArduPilot → MAVROS):"
if command -v ss &>/dev/null; then
    ss -ulnp 2>/dev/null | grep -E ":$MAVLINK_PORT |$MAVLINK_PORT " || echo "   (ss: no listener found)"
fi
echo "   MAVROS fcu_url udp://0.0.0.0:14551@ means MAVROS listens; ArduPilot connects."
echo ""

# 3. Do NOT send UDP to port 9002 - the ArduPilotPlugin parses all received
#    data as servo packets. Sending "test" causes "Incorrect protocol magic 25972".
echo "3. UDP: Do not send test packets to port $FDM_PORT - the ArduPilotPlugin"
echo "   interprets any received data as servo packets (causes protocol magic warning)."
echo ""

# 4. Docker-specific: if ArduPilot runs in BlueOS Docker
echo "4. Docker / BlueOS notes:"
echo "   - If Gazebo runs on HOST and ArduPilot in DOCKER on same machine:"
echo "     * ArduPilot must connect to HOST_IP:9002 (e.g. host.docker.internal or 172.17.0.1)"
echo "     * Gazebo plugin must listen on 0.0.0.0:9002 (not 127.0.0.1) to receive from Docker"
echo "   - If fdm_addr=127.0.0.1, plugin may only accept localhost; change to 0.0.0.0 for Docker."
echo "   - BlueOS ArduPilot must be started with: -M JSON -f JSON:HOST_IP"
echo "     where HOST_IP is the machine running Gazebo (e.g. 192.168.1.100 or host.docker.internal)"
echo ""

# 5. Quick checklist
echo "5. Checklist:"
echo "   [ ] Gazebo world loaded with orca4 model (ArduPilotPlugin active)"
echo "   [ ] BlueOS ArduPilot started with --model JSON or -M JSON"
echo "   [ ] ArduPilot JSON target = IP of machine running Gazebo"
echo "   [ ] Port 9002 reachable from ArduPilot container to Gazebo host"
echo "   [ ] MAVROS connects to ArduPilot (check /mavros/state)"
echo ""
echo "6. Verify MAVROS receives data:"
echo "   ros2 topic hz /mavros/imu/data"
echo "   ros2 topic hz /mavros/local_position/pose"
echo "   ros2 topic echo /mavros/imu/static_pressure  # depth"
echo "   If these don't update when moving the vehicle in Gazebo, FDM link is broken."
echo ""
