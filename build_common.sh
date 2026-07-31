#!/bin/bash
set -e

WS=/home/ws/ugv_ws
cd $WS || exit 1

PACKAGES=(
  cartographer
  costmap_converter_msgs
  costmap_converter
  emcl2
  explore_lite
  explore_lite_msgs
  gz_ros2_control
  openslam_gmapping
  slam_gmapping
  ldlidar
  rf2o_laser_odometry
  robot_pose_publisher
  teb_msgs
  teb_local_planner
  vizanti
  vizanti_cpp
  vizanti_demos
  vizanti_msgs
  vizanti_server
  ugv_msgs
  ugv_bringup
  ugv_chat_ai
  ugv_description
  ugv_gazebo
  ugv_nav
  ugv_slam
  ugv_tools
  ugv_vision
  ugv_voice
  ugv_web_app
)

echo "=============================="
echo "  Select packages to build"
echo "=============================="

for i in "${!PACKAGES[@]}"; do
  printf "[%2d] %s\n" $((i+1)) "${PACKAGES[$i]}"
done

echo
read -p "Please enter the package number to be compiled (space-separated): " SELECTION

SELECTED_PKGS=""

for index in $SELECTION; do
  pkg="${PACKAGES[$((index-1))]}"
  if [ -n "$pkg" ]; then
    SELECTED_PKGS="$SELECTED_PKGS $pkg"
  else
    echo "❌ Invalid number: $index"
    exit 1
  fi
done

echo
echo "✔ The following packages will be compiled.:"
echo "$SELECTED_PKGS"
echo

colcon build \
  --packages-select $SELECTED_PKGS \
  --symlink-install \
  --executor sequential

echo
echo "===== Build finished ====="
source install/setup.bash
echo "✔ Workspace sourced."
