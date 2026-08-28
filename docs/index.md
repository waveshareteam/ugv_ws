# ugv_ws Documentation

**ugv_ws** is a **ROS2 Humble** colcon workspace for **WaveShare UGV** mobile robots.
It connects **RViz2**, **SLAM**, and **Nav2** to real hardware over serial (ESP32 base board + LiDAR), with optional vision, [Web App](web_app.md), voice, and Gazebo simulation.

### Product names vs environment variables

WaveShare shop names and this repo use different labels for the same hardware:

| Your hardware (WaveShare) | `UGV_MODEL` | Notes |
|-----------------------------|-------------|-------|
| UGV Rover (6-wheel 4WD) | `ugv_rover` | e.g. [UGV Rover PT ROS2 Kit](https://www.waveshare.com/ugv-rover-pt-jetson-orin-ros2-kit.htm) |
| UGV Beast (tracked) | `ugv_beast` | e.g. [UGV Beast PT ROS2 Kit](https://www.waveshare.com/ugv-beast-pt-jetson-orin-ros2-kit.htm) |
| RaspRover (4WD) | `rasp_rover` | e.g. [RaspRover PT AI Kit](https://www.waveshare.com/rasprover.htm); add LiDAR for ROS2 |

| Shop kit type | What it includes |
|---------------|------------------|
| **AI Kit** | USB camera, pan-tilt, vision tutorials |
| **ROS2 Kit** | AI Kit + 360° LiDAR + OAK-D Lite + full SLAM / Nav2 stack |

**Click an image for full-screen view** — click outside, press **Esc**, or **×** to close.

<div class="img-row img-row-3 img-row-equal img-row-h-sm">

<figure>
<img class="img-zoom" alt="UGV Rover 6-wheel 4WD" src="https://github.com/user-attachments/assets/82144645-0d6a-436a-9536-5794b1bd4dd0" />
<figcaption>UGV Rover<br/><code>ugv_rover</code></figcaption>
</figure>

<figure>
<img class="img-zoom" alt="UGV Beast tracked" src="https://github.com/user-attachments/assets/6ad04d98-2e9b-43f2-96bc-4fd758d7fbaa" />
<figcaption>UGV Beast<br/><code>ugv_beast</code></figcaption>
</figure>

<figure>
<img class="img-zoom" alt="RaspRover 4WD" src="https://github.com/user-attachments/assets/2882f71e-82a6-4dff-ba2f-a66897e83355" />
<figcaption>RaspRover<br/><code>rasp_rover</code></figcaption>
</figure>

</div>

**ROS2 Kit** adds 360° LiDAR and OAK-D Lite on top of **AI Kit** hardware — set **`LDLIDAR_MODEL`** to match your sensor ([Installation — change model or LiDAR](installation.md#change-model-or-lidar)).

**Getting started** — read in this order ([details below](#suggested-reading-order)):

1. **[ROS2 Basics](ros2_basics.md)** — if you are new to ROS2 (topics, launch files used in this repo). Skip if you already know ROS2.
2. **[UGV Basics](ugv_basics.md)** — dual-controller layout, kit types, frames, factory Docker image.
3. **[Installation](installation.md)** — factory image or `build_first.sh`, set **`UGV_MODEL`** and **`LDLIDAR_MODEL`**.

Before use, set environment variables (pre-set in factory images, or via `build_first.sh` / `~/.bashrc`):

| Variable | Values | When | Role |
|----------|--------|------|------|
| `UGV_MODEL` | `ugv_rover`, `ugv_beast`, `rasp_rover` | **Always** — every launch | URDF / xacro, Gazebo model, wheel odom geometry |
| `LDLIDAR_MODEL` | `ld06`, `ld19`, `stl27l` | **Always** on real robot | LiDAR driver baud rate and launch include |
| `GZ_VERSION` | `classic`, `harmonic` | **Gazebo only** — set on **VM / desktop** | Classic vs Harmonic for [Gazebo](gazebo.md). Leave **unset** on the robot (**Pi / Jetson**) — do not run Gazebo there |

After install or editing `~/.bashrc`, verify:

```bash
echo $UGV_MODEL $LDLIDAR_MODEL $GZ_VERSION
```

`GZ_VERSION` may print empty when you do not use simulation — that is normal.

To change model or LiDAR later, edit `~/.bashrc` and `source ~/.bashrc`; you usually do not need to re-run full `build_first.sh` ([Installation — change model](installation.md#change-model-or-lidar)).

**Factory network (pre-built images):**

On a **physical robot**, SSH twice: port **22** is the **host** (run `ros2.sh` there); port **23** is the **ROS container** where you run tutorials (`root` / `ws`). Details: [UGV Basics — Factory Docker image](ugv_basics.md#factory-docker-image), [Installation](installation.md#factory-image-recommended).

| Service | Address |
|---------|---------|
| SSH — **host** (Raspberry Pi) | `<robot-ip>` port **22**, user `ws`, password `ws` |
| SSH — **host** (Jetson) | `<robot-ip>` port **22**, user `jetson`, password `jetson` |
| SSH — **container** (`ugv_ws`, ROS) | `<robot-ip>` port **23**, user `root`, password `ws` |
| Camera WebRTC | `http://<robot-ip>:8889/cam/` |
| Web App | `http://<robot-ip>:5100` (with **T0** robot stack running) |
| Web AI (experimental) | `http://<robot-ip>:5000` |

---

## Overview

Short map of what each tutorial chapter covers. Step-by-step guides are in the sidebar.

Most real-robot workflows use **T0** for the robot stack (bringup, SLAM, Nav2, or a demo launch) and **T1** for teleop or the [Web App](web_app.md). Optional **`explore_lite`** also runs in **T1** but only sends Nav2 goals while **T0** runs `nav.launch.py use_slam:=true` — see [Navigation — SLAM while navigating](navigation.md#slam-while-navigating). Only **one** **`/cmd_vel`** source at a time — see [Teleoperation — One motion source at a time](teleoperation.md#one-motion-source-at-a-time).

### [1. Robot Description](description.md)

URDF/xacro models, RViz configs, LiDAR and camera frames, TF diagrams.

- **`ugv_description`** — xacro for `ugv_rover`, `ugv_beast`, `rasp_rover`.
- **`display.launch.py`** — `robot_state_publisher` + RViz; visualize the model without powering motors.

### [2. Hardware Driver](bringup.md)

**`ugv_bringup`** — ROS2 bridge to the ESP32 motor board: motion, encoders, IMU, battery, pan-tilt, and LEDs.

- **`ugv_bringup`** node — UART **`/dev/ttyAMA0`** (115200); subscribes **`/cmd_vel`**, pan-tilt, LED topics.
- **`bringup_lidar.launch.py`** — `ldlidar` (**`/dev/ttyACM0`**) + RF2O + EKF → **`/scan`**, fused **`/odom`**.

2D/3D SLAM and Nav2 launches **include** this stack on hardware (or **`bringup_gazebo.launch.py`** when you pass **`use_sim_time:=true`**). [LiDAR](lidar.md) and [Vision](vision.md) demos are **hardware only** — they always use **`bringup_lidar`**, not Gazebo. Run **`bringup_lidar.launch.py`** alone for teleop-only, [Web App](web_app.md) teleop, or [Experimental](experimental.md) voice / Web AI.

### [3. Keyboard & Gamepad Control](teleoperation.md)

Manual driving via **`/cmd_vel`** — keyboard **or** gamepad in **T1**, not both.

- **`keyboard_ctrl`** — drive keys **latch** until **Space** / **`k`**; pan-tilt on **`0/1/2/r`**.
- **`teleop_twist_joy.launch.py`** — USB gamepad; right stick controls gimbal.

### [4. LiDAR Interaction](lidar.md)

Laser-based motion demos in **`ugv_slam`** — **`/scan`** (and **`/odom`** for avoidance) → **`/cmd_vel`**. Not SLAM or Nav2.

- **`demo.launch.py`** — includes bringup + one demo; **`exe:=lidar_follow`**, **`lidar_guard`**, or **`lidar_obstacle_avoidance`**.

### [5. Vision](vision.md)

USB camera and OAK-D Lite tracking in **`ugv_vision`**. **One camera type per session** — do not mix USB and OAK pipelines.

- **`demo.launch.py`** — includes bringup + vision node; USB demos also start **`camera.launch.py`**.
- Motion-tracking demos publish **`/cmd_vel`** — stop before SLAM, Nav2, or teleop.
- **`cam_webrtc`**, **`cam_oak_webrtc`**, … — browser preview at `http://<robot-ip>:8889/cam/`.

### [6. Mapping](mapping.md)

**`ugv_slam`** SLAM launches include bringup — **T0** SLAM + **T1** teleop while driving.

- **2D** — SLAM Toolbox, Gmapping, or Cartographer; save with repo-root **`save_map.sh`** → **`ugv_nav/maps/`**.
- **RTAB-Map** — 3D + OAK-D (Gazebo Harmonic: **`/oak/*`** via bridge; Classic: camera plugin); no **`save_map.sh`**; Nav2: **`use_localization:=rtabmap`**.
- 2D backends include **`robot_pose_publisher`** → **`/robot_pose`**.

### [7. Web App](web_app.md)

Optional Vizanti browser UI in **`ugv_web_app`** — does **not** start the robot stack; needs **T0** (bringup / SLAM / Nav) + **T1** `ros2 launch ugv_web_app bringup.launch.py`.

- Teleop, camera widgets, map / scan / TF, Nav2 goals, map save/load — depending on **T0**.
- Typical URL **`http://<robot-ip>:5100`** (rosbridge **5001**). Not the same as [Web AI](experimental.md#web-ai) on **5000**.

### [8. Navigation](navigation.md)

[Nav2](https://navigation.ros.org/) — **`nav.launch.py`** includes bringup + Nav2 + **`robot_pose_publisher`**. Default: navigate on a **saved map**; optional **`use_slam:=true`** for map-while-navigating (not the same flag as Mapping’s `use_slam:=sync`).

- Localization — **`amcl`** *(default)*, **`emcl`**, **`cartographer`**, **`slam_toolbox`**, **`rtabmap`** (OAK-D).
- Local planners — **`dwa`** *(default)*, **`teb`**, **`rpp`**, **`mppi`**. Prefer **AMCL** + **DWA** first; with **Cartographer** localization keep **DWA** — see [Navigation — Recommended combinations](navigation.md#recommended-combinations).
- Optional **`use_slam:=true`** on **`nav.launch.py`** — map while navigating (not **`use_localization:=slam_toolbox`** on a saved map — see [SLAM Toolbox: which path?](navigation.md#slam-toolbox-paths)); optional **`explore_lite`** in **T1**.
- Optional **`use_keepout_zones`**. Stop teleop and motion demos before Nav2.

### [9. Behavior Command Control](behavior_ctrl.md)

Scripted motion and map-point navigation via the **`/behavior`** action (**`behavior_ctrl`** in **`ugv_tools`**).

- Needs **T0** bringup (or SLAM / Nav) for **`/odom`**; map points need **`/robot_pose`**; **`pub_nav_point`** needs Nav2.
- Publishes **`/cmd_vel`** — stop other motion sources first.
- Web AI ([Experimental](experimental.md#web-ai)) talks to the same action; this chapter is CLI / JSON control.

### [10. Experimental](experimental.md)

Optional voice, Ollama LLM, and Web AI — not required for the core Mapping → Navigation path.

- **`ugv_voice`** — KWS / ASR / TTS / voice chat (no **`/cmd_vel`**). Wake words: [Experimental — Wake words](experimental.md#wake-words).
- **`ugv_chat_ai`** + **`behavior_ctrl`** — Web AI on **`:5000`** (LLM → motion). Separate from [Web App](web_app.md).

### [11. Gazebo](gazebo.md)

**`ugv_gazebo`** — Gazebo Classic or GZ Harmonic (`GZ_VERSION`); **VM or desktop only** — do **not** run on the Pi/Jetson on the physical UGV.

- **`bringup_gazebo.launch.py`** — simulated robot + `ros2_control` (+ optional RViz).
- Add **`use_sim_time:=true`** on **SLAM and Nav2** launches (Mapping / Navigation tutorial parity in [Gazebo](gazebo.md)).
- [LiDAR Interaction](lidar.md) and [Vision](vision.md) demos are **hardware only** — run on the real robot, not in Gazebo.

Developer package lookup: [Package Reference](packages.md).

---

## Typical paths

Use separate terminals for real-robot workflows. Factory images: SSH into the container first ([Installation](installation.md)). Only **one** **`/cmd_vel`** source at a time.

| Goal | Commands |
|------|----------|
| View URDF only | **T0:** `ros2 launch ugv_description display.launch.py use_rviz:=true` |
| Boot real robot | **T0:** `ros2 launch ugv_bringup bringup_lidar.launch.py use_rviz:=true` |
| Teleop (keyboard) | **T0:** bringup · **T1:** `ros2 run ugv_tools keyboard_ctrl` |
| Teleop (gamepad) | **T0:** bringup · **T1:** `ros2 launch ugv_tools teleop_twist_joy.launch.py` |
| LiDAR demo | **T0:** `ros2 launch ugv_slam demo.launch.py exe:=lidar_follow use_rviz:=true` |
| USB camera + WebRTC | **T0:** `ros2 launch ugv_vision demo.launch.py exe:=cam_webrtc use_rviz:=true` · browser `http://<ip>:8889/cam/` |
| OAK-D preview | **T0:** `ros2 launch ugv_vision demo.launch.py exe:=cam_oak_webrtc use_rviz:=true` · browser `http://<ip>:8889/cam/` |
| Map (SLAM Toolbox) | **T0:** `ros2 launch ugv_slam slam_toolbox.launch.py use_slam:=sync use_rviz:=true` · **T1:** `keyboard_ctrl` · **T2:** `./save_map.sh` → **`3`** |
| Map (Gmapping) | **T0:** `ros2 launch ugv_slam gmapping.launch.py use_rviz:=true` · **T1:** `keyboard_ctrl` · **T2:** `./save_map.sh` → **`1`** |
| Map (Cartographer) | **T0:** `ros2 launch ugv_slam cartographer.launch.py use_rviz:=true` · **T1:** `keyboard_ctrl` · **T2:** `./save_map.sh` → **`2`** |
| Map (RTAB-Map) | **T0:** `ros2 launch ugv_slam rtabmap.launch.py use_rviz:=true` · **T1:** `keyboard_ctrl` — no **`save_map.sh`** |
| Web App (teleop / viz) | **T0:** bringup (or SLAM / Nav) · **T1:** `ros2 launch ugv_web_app bringup.launch.py` · browser `http://<ip>:5100` |
| Nav (AMCL + DWA) | **T0:** `ros2 launch ugv_nav nav.launch.py use_rviz:=true` (after **`save_map.sh`**) |
| Nav (SLAM + Nav) | **T0:** `ros2 launch ugv_nav nav.launch.py use_rviz:=true use_slam:=true` · **T1** *(optional)*: `ros2 launch explore_lite explore.launch.py` · **T2:** `./save_map.sh` → **`3`** |
| Nav (RTAB-Map) | **T0:** `ros2 launch ugv_nav nav.launch.py use_rviz:=true use_localization:=rtabmap` |
| Behavior commands | **T0:** bringup (or SLAM / Nav) · **T1:** `ros2 run ugv_tools behavior_ctrl` — [Behavior Command Control](behavior_ctrl.md) |
| Web AI | **T0:** bringup · **T1:** `ros2 run ugv_tools behavior_ctrl` · **T2:** `ros2 run ugv_chat_ai app --ros-args -p server_url:=http://<ollama-ip>:11434/api/chat` · browser `http://<ip>:5000` |
| Simulation only *(VM / desktop)* | **T0:** `ros2 launch ugv_gazebo bringup_gazebo.launch.py use_rviz:=true`; SLAM/Nav add `use_sim_time:=true` — not on the physical robot |

---

## Suggested reading order

| Step | Page | You learn |
|------|------|-----------|
| 1 | [ROS2 Basics](ros2_basics.md) | ROS2 vocabulary (skip if you already know ROS2) |
| 2 | [UGV Basics](ugv_basics.md) | Dual-controller layout, frames, kit types |
| 3 | [Installation](installation.md) | Factory Docker or build **`ugv_ws`**, set env vars |
| 4 | [Robot Description](description.md) | Model, sensors, TF |
| 5 | [Hardware Driver](bringup.md) | Serial ports, `/scan`, `/odom` |
| 6 | [Keyboard & Gamepad Control](teleoperation.md) | Drive manually (keys latch until stop) |
| 7 | [LiDAR Interaction](lidar.md) | Laser follow / guard / avoid |
| 8 | [Vision](vision.md) | USB and OAK-D tracking via `demo.launch.py` |
| 9 | [Mapping](mapping.md) | SLAM + save map |
| 10 | [Web App](web_app.md) | Browser teleop / viz (optional) |
| 11 | [Navigation](navigation.md) | Nav2 on saved map |
| 12 | [Behavior Command Control](behavior_ctrl.md) | Scripted `/behavior` motion and map points |
| 13 | [Experimental](experimental.md) | Voice, Ollama, Web AI (optional) |
| 14 | [Gazebo](gazebo.md) | Simulation without hardware |
