#!/bin/bash
# health_check.sh — verify all UGV sensor topics are alive at expected rates.
# Run inside the container after bringup:
#   ros2 launch ugv_bringup bringup_full.launch.py &
#   sleep 10 && ./scripts/health_check.sh

SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="/home/ws/ugv_ws/install/setup.bash"

source "$SETUP" 2>/dev/null
source "$WS_SETUP" 2>/dev/null

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

PASS=0
FAIL=0

check_topic() {
    local topic="$1"
    local min_rate="$2"
    local label="$3"

    # Wait up to 5s for at least one message
    if ! timeout 5 ros2 topic echo --once "$topic" > /dev/null 2>&1; then
        echo -e "  ${RED}FAIL${NC}  $label ($topic) — no messages received"
        FAIL=$((FAIL + 1))
        return
    fi

    # Sample rate over 5 seconds
    rate=$(timeout 6 ros2 topic hz "$topic" 2>/dev/null \
           | grep "average rate" | tail -1 \
           | awk '{print $3}' | tr -d ':')

    if [ -z "$rate" ]; then
        echo -e "  ${YELLOW}WARN${NC}  $label ($topic) — alive but could not measure rate"
        PASS=$((PASS + 1))
        return
    fi

    # Compare rate (integer comparison)
    rate_int=${rate%.*}
    if [ "$rate_int" -ge "$min_rate" ] 2>/dev/null; then
        echo -e "  ${GREEN}PASS${NC}  $label ($topic) @ ${rate} Hz"
        PASS=$((PASS + 1))
    else
        echo -e "  ${RED}FAIL${NC}  $label ($topic) @ ${rate} Hz (expected >= ${min_rate} Hz)"
        FAIL=$((FAIL + 1))
    fi
}

echo ""
echo "=== UGV Health Check ==="
echo ""
echo "--- Base platform ---"
check_topic /voltage          18  "ESP32 voltage"
check_topic /imu/data_raw     18  "IMU raw"
check_topic /odom/odom_raw    18  "Wheel odometry raw"
check_topic /odom             8   "Odometry"

echo ""
echo "--- Lidar ---"
check_topic /scan             8   "LD19 laser scan"

echo ""
echo "--- Cameras ---"
check_topic /image_raw        25  "USB webcam"
check_topic /oak/rgb/image_raw    25  "OAK-D RGB"
check_topic /oak/stereo/image_raw 15  "OAK-D depth"

echo ""
echo "--- TF ---"
check_topic /tf               1   "Transform tree"

echo ""
echo "========================"
if [ "$FAIL" -eq 0 ]; then
    echo -e "  ${GREEN}All ${PASS} checks passed${NC}"
else
    echo -e "  ${GREEN}${PASS} passed${NC}  ${RED}${FAIL} failed${NC}"
fi
echo ""
