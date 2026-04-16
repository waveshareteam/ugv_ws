#!/bin/bash
# =============================================================================
# test_cameras.sh — verify USB webcam and OAK-D Lite are publishing.
#
# PREREQUISITES
#   Camera nodes must be running.  Either standalone:
#     ros2 launch ugv_vision camera.launch.py       # USB webcam only
#     ros2 launch ugv_vision oak_d_lite.launch.py   # OAK-D only
#   Or via the full stack (starts both):
#     ros2 launch ugv_bringup bringup_full.launch.py
#
#   Both physical devices must be passed through to the Docker container.
#   Check your docker run command includes:
#     --device /dev/video0              ← USB webcam  (0bda:5842)
#     --device /dev/bus/usb             ← OAK-D Lite  (03e7:2485, Movidius MyriadX)
#
#   The OAK-D also needs a udev rule on the HOST (not in the container):
#     echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' \
#       | sudo tee /etc/udev/rules.d/80-movidius.rules
#     sudo udevadm control --reload-rules && sudo udevadm trigger
#   (Only needs to be done once per host OS install.)
#
# WHAT IT DOES
#   For each camera, confirms at least one message arrives within 5s,
#   then measures the publish rate over 5s and reports pass/fail.
#
# EXPECTED RESULTS
#   USB webcam    /image_raw              ~30 Hz  (640x480 MJPEG)
#   OAK-D RGB     /oak/rgb/image_raw      ~30 Hz  (USB3 Super Speed)
#   OAK-D depth   /oak/stereo/image_raw   ~20 Hz  (stereo matching is heavier)
#
#   USB scheduling jitter of ~133ms is normal on all three — not a fault.
# =============================================================================

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
