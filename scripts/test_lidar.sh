#!/bin/bash
# test_lidar.sh — verify LD19 lidar is publishing scan data.
# Run inside the container with ldlidar already running.

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
