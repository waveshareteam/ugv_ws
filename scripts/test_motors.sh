#!/bin/bash
# test_motors.sh — drive forward briefly, then stop.
# Confirms cmd_vel → ESP32 → wheels path is working.
# Run inside the container with ugv_bringup already running.
#
# Usage:
#   ./scripts/test_motors.sh              # default: 0.1 m/s for 2s
#   ./scripts/test_motors.sh 0.2 3        # custom: speed m/s, duration s

SPEED="${1:-0.1}"
DURATION="${2:-2}"

source /opt/ros/humble/setup.bash 2>/dev/null
source /home/ws/ugv_ws/install/setup.bash 2>/dev/null

RATE=10
TICKS=$(echo "$DURATION * $RATE" | bc)

echo ""
echo "=== Motor Test ==="
echo "Speed: ${SPEED} m/s  Duration: ${DURATION}s"
echo ""

# Confirm ESP32 is alive before moving
if ! timeout 3 ros2 topic echo --once /voltage > /dev/null 2>&1; then
    echo "ERROR: /voltage not publishing — is ugv_bringup running?"
    exit 1
fi

echo "Driving forward..."
ros2 topic pub --times "$TICKS" /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: ${SPEED}}, angular: {z: 0.0}}" --rate $RATE

echo "Stopping..."
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0}, angular: {z: 0.0}}"

echo "Done."
echo ""
