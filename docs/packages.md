# Package Reference

Lookup reference for workspace layout and package roles. For a guided tutorial, start at [index](index.md) and follow the [suggested reading order](index.md#suggested-reading-order).

---

## Workspace layout

```text
ugv_ws
|-- README.md
|-- docs/                   # MkDocs documentation
|-- build_common.sh         # Selective package build
|-- build_first.sh          # First-time build and environment setup
|-- requirements.txt        # Python dependencies
|-- ros2.sh                 # Container and SSH setup
|-- save_map.sh             # Save SLAM maps
|-- src
    |-- ugv_else            # Third-party ROS2 packages
    |-- ugv_main            # Core UGV packages
```

---

## Root scripts

| File | Purpose |
|------|---------|
| `build_first.sh` | Initial dependency install, model selection, optional Gazebo, full workspace build |
| `build_common.sh` | Interactive selective `colcon` build |
| `requirements.txt` | Python packages for vision, voice, and AI features |
| `ros2.sh` | Container lifecycle and SSH on robot or VM |
| `save_map.sh` | Save maps from gmapping, Cartographer, or slam_toolbox |

---

## ugv_else (third-party)

Auxiliary packages for LiDAR drivers, odometry, mapping, localization, planning, and web visualization.

| Package | Description |
|---------|-------------|
| `cartographer` | 2D SLAM and pure localization |
| `costmap_converter` | Convert costmaps to polygons for local planners |
| `emcl2` | EMCL Monte Carlo localization |
| `explore_lite` | Autonomous frontier exploration — use with Nav2 **`use_slam:=true`**; see [Navigation — SLAM while navigating](navigation.md#slam-while-navigating) |
| `openslam_gmapping` / `slam_gmapping` | 2D SLAM with GMapping |
| `gz_ros2_control` | `ros2_control` integration with Gazebo Harmonic |
| `ldlidar` | LD-series LiDAR driver |
| `rf2o_laser_odometry` | Laser odometry (RF2O) |
| `robot_pose_publisher` | Publish robot pose from TF |
| `teb_local_planner` | Timed Elastic Band local planner |
| `vizanti` | Web visualization and control UI |

---

## ugv_main (core)

| Package | Description |
|---------|-------------|
| `ugv_bringup` | Motor control, wheel odometry, LiDAR bringup |
| `ugv_chat_ai` | Flask web app with LLM task control |
| `ugv_description` | Xacro URDF and mesh files |
| `ugv_gazebo` | Gazebo world and simulation launch files |
| `ugv_msgs` | Custom messages, services, and actions |
| `ugv_nav` | Nav2 configuration, maps, launch files |
| `ugv_slam` | SLAM launch files; **`demo.launch.py`** (bringup + LiDAR demo); `lidar_follow`, `lidar_guard`, `lidar_obstacle_avoidance` |
| `ugv_tools` | Keyboard/gamepad teleop, behavior control |
| `ugv_vision` | **`demo.launch.py`** (bringup + vision node; USB also starts `camera.launch.py`; OAK opens camera in-node); tracking, WebRTC preview |
| `ugv_voice` | KWS / ASR / TTS / voice chat; wake-word lists in [Experimental](experimental.md#wake-words) |
| `ugv_web_app` | Vizanti web app — teleop, visualization, Nav widgets ([Web App](web_app.md)) |

---

## Related Tutorials

| Chapter | What it adds |
|---------|----------------|
| [index](index.md) | Overview and typical launch paths |
| [Hardware Driver](bringup.md) | `ugv_bringup` in practice |
| [Navigation](navigation.md) | `ugv_nav` launch arguments |
