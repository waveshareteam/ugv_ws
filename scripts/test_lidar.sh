#!/bin/bash
# =============================================================================
# test_lidar.sh — verify LD19 lidar is publishing scan data.
#
# PREREQUISITES
#   ldlidar must be running.  Either standalone:
#     ros2 launch ldlidar ldlidar.launch.py
#   Or via the full stack:
#     ros2 launch ugv_bringup bringup_full.launch.py
#
#   The lidar must be wired to the UART configured in /boot/firmware/config.txt:
#     dtoverlay=uart1-pi5,pins_32_33  →  /dev/ttyAMA1  (default)
#   And that device must be passed through to the container:
#     docker run ... --device /dev/ttyAMA1
#   To use a different port:
#     export LDLIDAR_PORT=/dev/ttyAMA2 before launching
#
# WHAT IT DOES
#   1. Waits up to 5s for the first /scan message
#   2. Samples the publish rate for 10s and prints the result
#
# EXPECTED RESULT
#   ~10 Hz with low jitter (std dev < 5ms).
#   Occasional 200ms gaps are normal — that's the lidar completing a full
#   rotation where the timing is slightly non-uniform.
# =============================================================================

source /opt/ros/humble/setup.bash 2>/dev/null
source /home/ws/ugv_ws/install/setup.bash 2>/dev/null

echo ""
echo "=== Lidar Test (10s sample) ==="
echo ""

if ! timeout 5 ros2 topic echo --once /scan > /dev/null 2>&1; then
    echo "ERROR: /scan not publishing."
    echo "  Is ldlidar running?  Check: ros2 launch ldlidar ldlidar.launch.py"
    echo "  Is /dev/ttyAMA1 passed through to the container?"
    echo "    docker run ... --device /dev/ttyAMA1"
    exit 1
fi

echo "Scan data confirmed. Measuring rate for 10s..."
echo ""
timeout 11 ros2 topic hz /scan 2>/dev/null | grep -v "^$"

echo ""
echo "Expected: ~10 Hz"
echo ""
