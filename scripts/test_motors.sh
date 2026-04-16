#!/bin/bash
# =============================================================================
# test_motors.sh — drive forward briefly, then stop.
#
# PREREQUISITES
#   ugv_bringup must be running (does not need lidar or cameras):
#     ros2 run ugv_bringup ugv_bringup
#   OR via the full stack:
#     ros2 launch ugv_bringup bringup_full.launch.py
#
#   Make sure the robot has clear space in front of it before running.
#
# WHAT IT DOES
#   1. Checks /voltage to confirm ESP32 serial comms are alive
#   2. Publishes cmd_vel at the given speed for the given duration
#   3. Sends an explicit stop command
#
#   The cmd_vel watchdog in ugv_bringup will also stop the motors
#   automatically if commands stop arriving (0.5s timeout), so the
#   robot will not run away if this script is killed mid-test.
#
# USAGE
#   ./scripts/test_motors.sh              # default: 0.1 m/s for 2s
#   ./scripts/test_motors.sh 0.2 3        # 0.2 m/s for 3s
#
# EXPECTED RESULT
#   Robot drives straight forward and stops cleanly.
#   If the wheels don't turn, check /voltage is publishing and that
#   the ESP32 init sequence completed (look for "ugv_bringup ready"
#   in the bringup terminal).
# =============================================================================

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
