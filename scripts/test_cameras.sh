#!/bin/bash
# test_cameras.sh — verify USB webcam and OAK-D Lite are publishing.
# Run inside the container with camera nodes already running.

source /opt/ros/humble/setup.bash 2>/dev/null
source /home/ws/ugv_ws/install/setup.bash 2>/dev/null

GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m'

echo ""
echo "=== Camera Test ==="
echo ""

# --- USB webcam ---
echo "--- USB Webcam ---"
echo "  Device:  /dev/video0"
echo "  Topic:   /image_raw"
if timeout 5 ros2 topic echo --once /image_raw > /dev/null 2>&1; then
    rate=$(timeout 6 ros2 topic hz /image_raw 2>/dev/null \
           | grep "average rate" | tail -1 | awk '{print $3}' | tr -d ':')
    echo -e "  ${GREEN}OK${NC} — ${rate:-?} Hz (expected ~30 Hz)"
else
    echo -e "  ${RED}FAIL${NC} — no data on /image_raw"
    echo "  Is camera.launch.py running?  Is /dev/video0 in the container?"
fi

echo ""

# --- OAK-D Lite RGB ---
echo "--- OAK-D Lite RGB ---"
echo "  Bus:     USB3 (03e7:2485)"
echo "  Topic:   /oak/rgb/image_raw"
if timeout 5 ros2 topic echo --once /oak/rgb/image_raw > /dev/null 2>&1; then
    rate=$(timeout 6 ros2 topic hz /oak/rgb/image_raw 2>/dev/null \
           | grep "average rate" | tail -1 | awk '{print $3}' | tr -d ':')
    echo -e "  ${GREEN}OK${NC} — ${rate:-?} Hz (expected ~30 Hz)"
else
    echo -e "  ${RED}FAIL${NC} — no data on /oak/rgb/image_raw"
    echo "  Is oak_d_lite.launch.py running?"
    echo "  Are USB devices passed through? (--device /dev/bus/usb)"
    echo "  Udev rule needed on host:"
    echo "    echo 'SUBSYSTEM==\"usb\", ATTRS{idVendor}==\"03e7\", MODE=\"0666\"' \\"
    echo "      | sudo tee /etc/udev/rules.d/80-movidius.rules"
    echo "    sudo udevadm control --reload-rules && sudo udevadm trigger"
fi

echo ""

# --- OAK-D Lite Depth ---
echo "--- OAK-D Lite Depth ---"
echo "  Topic:   /oak/stereo/image_raw"
if timeout 5 ros2 topic echo --once /oak/stereo/image_raw > /dev/null 2>&1; then
    rate=$(timeout 6 ros2 topic hz /oak/stereo/image_raw 2>/dev/null \
           | grep "average rate" | tail -1 | awk '{print $3}' | tr -d ':')
    echo -e "  ${GREEN}OK${NC} — ${rate:-?} Hz (expected ~20 Hz)"
else
    echo -e "  ${RED}FAIL${NC} — no data on /oak/stereo/image_raw"
fi

echo ""
