# Gazebo

Run the same tutorial flow in **Gazebo** on the factory **VM image** or a **desktop** with simulation installed — **simulation only**.

For real-robot bringup, see [Hardware Driver](bringup.md).

!!! warning "Do not run Gazebo on the physical robot"
    **Do not** launch **`bringup_gazebo.launch.py`** or any Gazebo workflow on the **Pi/Jetson mounted on the real UGV**. Gazebo is heavy (CPU/GPU, X11) and is meant for a VM or desktop. On hardware, use **`bringup_lidar.launch.py`** — [Hardware Driver — Launch](bringup.md#launch-physical-robot).

Never run Gazebo bringup and **`bringup_lidar.launch.py`** at the same time.

## Prerequisites

1. **Gazebo is optional.** Skip it in `build_first.sh` (default) unless you need this chapter. On a VM / desktop, choose Classic or Harmonic ([Installation](installation.md)).
2. **`GZ_VERSION`** set in `~/.bashrc` (`classic` or `harmonic`) **only if** Gazebo was installed. Empty is normal on the robot.
3. **`UGV_MODEL`** matches your kit.

!!! warning "Safety"
    Simulation still drives the virtual robot with **`/cmd_vel`** (teleop, SLAM mapping, Nav2). Keep the sim view clear of obstacles you care about; stop teleop or send zero velocity before switching tasks:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist --once
```

Do not run **`bringup_lidar.launch.py`** (real hardware) and Gazebo bringup at the same time.

---

## Overview

### Before you start

| Do | Do not |
|----|--------|
| Run Gazebo on a **VM or desktop** only | Run **`bringup_gazebo.launch.py`** on the **physical robot** (Pi/Jetson on the UGV) — use [Hardware Driver](bringup.md) instead |
| Add **`use_sim_time:=true`** on SLAM and Nav2 launches in sim | Forget **`use_sim_time`** — sensors and TF will look frozen |
| Use **`bringup_gazebo.launch.py`** (or SLAM / Nav launches that include it) for sim | Run **`bringup_lidar.launch.py`** (real serial) at the same time |
| Stop teleop / motion demos before Nav2 in sim | Mix live **`/cmd_vel`** sources — see [Teleoperation — One motion source at a time](teleoperation.md#one-motion-source-at-a-time) |

[Mapping](mapping.md) and [Navigation](navigation.md) SLAM / Nav launches **include Gazebo bringup** — do not run **`bringup_gazebo.launch.py`** separately for those workflows.

### Simulation setup

| Item | Detail |
|------|--------|
| VM image | [VM_ROS2 VirtualBox](https://drive.google.com/file/d/1BUiWwmoEM_r46liVtBiZyStXq5lhEM2j/view?usp=sharing) |
| Gazebo | **Classic** (`GZ_VERSION=classic`) or **Harmonic** (`GZ_VERSION=harmonic`) |
| Key flag | `use_sim_time:=true` on SLAM and Nav2 launches |

VM images include both backends; `~/.bashrc` selects which `ugv_gazebo` uses.

On the VM **host** before GUI:

```bash
xhost +
cd /home/ws/ugv_ws && bash ros2.sh
```

---

## Boot simulated robot

**VM or desktop only** — not on the physical UGV.

```bash
ros2 launch ugv_gazebo bringup_gazebo.launch.py use_rviz:=true
```

Replaces `bringup_lidar.launch.py` for all sim workflows. No serial driver — do not run real bringup at the same time.

If the program no longer needs to run, please use **`Ctrl+C`** to close the running session.

---

## Tutorial parity

Same workflow as [Mapping](mapping.md) and [Navigation](navigation.md): **T0** launch (with **`use_sim_time:=true`**) + **T1** teleop while mapping. SLAM and Nav launches **include Gazebo bringup** — do not run **`bringup_gazebo.launch.py`** separately for those workflows.

**T1** teleop (while **T0** SLAM is running):

| Input | Command |
|-------|---------|
| Keyboard | `ros2 run ugv_tools keyboard_ctrl` |
| Gamepad | `ros2 launch ugv_tools teleop_twist_joy.launch.py` — default **`angular_speed_limit:=1.0`**; lower if turns feel too fast |

### Mapping

| Backend | **T0** | Save / notes |
|---------|--------|--------------|
| SLAM Toolbox | `ros2 launch ugv_slam slam_toolbox.launch.py use_slam:=sync use_rviz:=true use_sim_time:=true` | **`save_map.sh`** option **`3`** — [Mapping — SLAM Toolbox](mapping.md#slam-toolbox) |
| Gmapping | `ros2 launch ugv_slam gmapping.launch.py use_rviz:=true use_sim_time:=true` | **`save_map.sh`** option **`1`** — [Mapping — Gmapping](mapping.md#gmapping) |
| Cartographer | `ros2 launch ugv_slam cartographer.launch.py use_rviz:=true use_sim_time:=true` | **`save_map.sh`** option **`2`** — [Mapping — Cartographer](mapping.md#cartographer) |
| RTAB-Map | `ros2 launch ugv_slam rtabmap.launch.py use_rviz:=true use_sim_time:=true` | No **`save_map.sh`**; uses Gazebo **`/oak/*`** + **`/scan`** (no OAK hardware) — [Mapping — RTAB-Map](mapping.md#rtab-map). **Harmonic:** via **`ros_gz_bridge`**; **Classic:** camera plugin. Optional: **`use_viz:=true`**. Sim uses Gazebo odom TF — do not set **`use_odom:=icp|rgbd`** (conflicts; those options are for hardware only). |

Verify **`/scan`** (all backends) before **T1** teleop. RTAB-Map: also check **`/oak/image_raw`**.

**T2** (2D only, while **T0** still running): **`./save_map.sh`** — see [Save maps](#save-maps).

### Navigation

For a **saved map** (2D **`map.yaml`** or RTAB-Map session / database). For **map while navigating**, use **`use_slam:=true`** — no prior **`save_map.sh`**; see [Navigation — SLAM while navigating](navigation.md#slam-while-navigating).

| Mode | **T0** | Notes |
|------|--------|-------|
| 2D (AMCL default) | `ros2 launch ugv_nav nav.launch.py use_rviz:=true use_sim_time:=true` | After 2D SLAM + **`save_map.sh`** — [Navigation](navigation.md) |
| SLAM + Nav | `ros2 launch ugv_nav nav.launch.py use_rviz:=true use_sim_time:=true use_slam:=true` | **T1** *(optional)*: `ros2 launch explore_lite explore.launch.py use_sim_time:=true` · **T2:** `./save_map.sh` → **`3`** — [Navigation — SLAM while navigating](navigation.md#slam-while-navigating) |
| RTAB-Map | `ros2 launch ugv_nav nav.launch.py use_rviz:=true use_sim_time:=true use_localization:=rtabmap` | **`view_nav_3d.rviz`** — [Navigation — RTAB-Map](navigation.md#rtab-map) |

Stop teleop and motion demos before Nav2.

---

## Save maps

| Backend | In simulation |
|---------|----------------|
| SLAM Toolbox, Gmapping, Cartographer | Same **`save_map.sh`** as hardware while **T0** is still running — [Mapping — Save the map](mapping.md#save-the-map) |
| RTAB-Map | **Not** **`save_map.sh`** — 3D database in session; see [Mapping — RTAB-Map](mapping.md#rtab-map) |

---

## Troubleshooting

| Symptom | Likely cause | What to try |
|---------|--------------|-------------|
| Robot or sensors frozen in RViz | Forgot **`use_sim_time:=true`** on SLAM / Nav | Add flag to **T0** launch; restart Gazebo bringup |
| Gazebo window does not open | No X11 / `xhost` on VM host | Run **`xhost +`** on host; launch from **`ros2.sh`** container |
| Empty **`/scan`** in sim | Gazebo bringup not running or wrong model | Use SLAM/Nav launch (includes sim bringup) or **`bringup_gazebo.launch.py`** alone first |
| Real robot + sim both moving | Mixed bringup | Stop hardware **`bringup_lidar`** — sim uses **`bringup_gazebo`** only |
| Gazebo on Pi/Jetson / UGV | Wrong machine | Run Gazebo on VM or desktop only; on hardware use [Hardware Driver](bringup.md) |
| RTAB-Map: no depth in sim | Camera bridge not up | Check **`/oak/image_raw`**; see [Mapping — RTAB-Map](mapping.md#rtab-map) |
| Teleop turns too fast / slow in sim | Angular speed cap | Gamepad: change **`angular_speed_limit`** (default **`1.0`**) — see [Tutorial parity](#tutorial-parity) |

---

## Related Tutorials

| Chapter | What it adds |
|---------|----------------|
| [Installation](installation.md) | Optional Gazebo via `build_first.sh` (default: skip) |
| [Mapping](mapping.md) | SLAM backends + `use_sim_time:=true` |
| [Navigation](navigation.md) | Nav2 in sim |
| [Hardware Driver](bringup.md) | Real robot (do not mix with Gazebo bringup) |

**Next:** deploy on hardware without `use_sim_time`.
