# Installation

Install **`ugv_ws`** on **Ubuntu 22.04** with **ROS2 Humble**.

Most users start with the **factory Docker image** on the robot or VM — no compile step. Developers **clone on the host** (install **git-lfs** and run **`git lfs pull`** there), then run **`build_first.sh`** **inside the container** to install dependencies, set `UGV_MODEL` / `LDLIDAR_MODEL`, and run `colcon build`. Gazebo is optional — skip it unless you need [simulation](gazebo.md).

**Factory image:** workflow depends on hardware — **robot** (Pi / Jetson) uses SSH into the container; **VM** uses two local terminals (`xhost +` + `ros2.sh`). See [Factory image](#factory-image-recommended) below.

---

## Factory image (recommended)

Most kits ship a **pre-built Docker container** with `UGV_MODEL`, `LDLIDAR_MODEL`, and ROS2 already configured. How you enter the container differs between a **physical robot** and a **VM**.

### Robot (Raspberry Pi / Jetson)

On the robot, `ros2.sh` **starts** the container on the host; you then **SSH into the container** from your PC (port **23**).

**1. SSH to the robot host** (port **22**):

| Host board | Host name | SSH user | Password |
|------------|-----------|----------|----------|
| Raspberry Pi 4B / 5 | *(IP or mDNS)* | `ws` | `ws` |
| Jetson Orin Nano | **`jetson`** | `jetson` | `jetson` |

**2. On the host**, start the container:

```bash
cd /home/ws/ugv_ws && bash ros2.sh
```

Choose **Enter container** — on ARM, the script starts Docker and exits; connect via SSH next.

**3. SSH into the ROS container** from your PC:

| Field | Value |
|-------|-------|
| Host | Robot IP address |
| Port | **23** |
| User | `root` |
| Password | `ws` |

**4. Inside the container:**

```bash
cd /home/ws/ugv_ws
source ~/.bashrc
echo $UGV_MODEL $LDLIDAR_MODEL
```

More host/container steps: [UGV Basics — Factory Docker image](ugv_basics.md#factory-docker-image).

---

### VM (VirtualBox / x86)

On the factory **VM**, open **two terminal windows** on the VM desktop (host setup — not tutorial **T0**/**T1**, which start after you are inside the container):

**First window — allow GUI apps (RViz / Gazebo) to display:**

```bash
xhost +
```

Leave this window open while using GUI tools.

**Second window — enter the container:**

```bash
cd /home/ws/ugv_ws && bash ros2.sh
```

Choose **Enter container** — on x86, `ros2.sh` runs **`docker exec`** and drops you into a shell **inside** the container (no port 23).

Then inside the container:

```bash
cd /home/ws/ugv_ws
source ~/.bashrc
echo $UGV_MODEL $LDLIDAR_MODEL
```

---

### `ros2.sh` by platform

| Platform | Container | How you enter |
|----------|-----------|---------------|
| x86 / VM | `ros_humble` | `docker exec` via `ros2.sh` (local terminal) |
| Raspberry Pi | `ugv_rpi_ros_humble` | SSH port **23** after `ros2.sh` on host |
| Jetson | `ugv_jetson_ros_humble` | SSH port **23** after `ros2.sh` on host |

On ARM, `ros2.sh` stops `ugv-app`, `ugv-jupyter`, and `roarm_web_app` host services before you SSH into the container.

`UGV_MODEL` and `LDLIDAR_MODEL` are pre-set in factory images.

### Factory downloads

- [VM_ROS2 VirtualBox image](https://drive.google.com/file/d/1BUiWwmoEM_r46liVtBiZyStXq5lhEM2j/view?usp=sharing)
- [UGV Rover PI ROS2 Wiki](https://www.waveshare.com/wiki/UGV_Rover_PI_ROS2)
- [UGV Beast PI ROS2 Wiki](https://www.waveshare.com/wiki/UGV_Beast_PI_ROS2)

VM images include Gazebo Classic and Harmonic; `GZ_VERSION` in `~/.bashrc` selects which `ugv_gazebo` uses.

**Next (factory image):** [Hardware Driver](bringup.md) or [Robot Description](description.md).

---

## Build from source

Typical flow: **clone on the host**, then run **`build_first.sh` inside Docker** (container user is **root**). Do **Git LFS on the host** as the user who cloned — not as root in the container, or `.git` / model files become root-owned and the host checkout breaks.

### Clone on the host (Git LFS)

Voice / vision weights are **Git LFS** files. Install **git-lfs** and pull them **when you download the code**, on the **host**, **without sudo**:

```bash
sudo apt update
sudo apt install -y git git-lfs
git lfs install
git clone -b ros2-humble-develop-251125 https://github.com/waveshareteam/ugv_ws.git
cd ugv_ws
git lfs pull
```

`build_first.sh` does **not** run `git lfs pull`. If you skip this step, ASR/TTS (and some vision weights) are pointer files and those nodes fail.

Tutorials assume **`ugv_ws`** at **`/home/ws/ugv_ws`**. If you cloned elsewhere, adjust paths in `~/.bashrc` later.

### Initial setup with `build_first.sh`

Run this **inside the container** (or on a native Humble install). Do not run `git lfs pull` here as root.

```bash
cd /home/ws/ugv_ws
sudo chmod +x build_first.sh
sudo bash build_first.sh
```

The script prints **`[1/6]` … `[6/6]`**. Overview:

| Step | What happens | You choose? |
|------|----------------|-------------|
| **[1/6]** | Basic apt deps (pip, colcon-argcomplete, screen, speech, GStreamer RTSP, …) | — |
| *(after 1)* | Optional **`pip install -r requirements.txt`** | **y** / **N** |
| **[2/6]** | ROS2 Humble desktop, Nav2, Cartographer, RTAB-Map, depthai, v4l2_camera, … | — |
| *(after 2)* | **Gazebo** — default **Skip** (`0`); Classic / Harmonic only if you need sim | **Enter** / **0–2** |
| **[3/6]** | Append `source /opt/ros/humble/setup.bash` to `~/.bashrc` | — |
| **[4/6]** | **`UGV_MODEL`** + **`LDLIDAR_MODEL`** (select both) | yes |
| *(prompt)* | Save model / LiDAR to **`~/.bashrc`?** | **y** / **N** |
| **[5/6]** | **`colcon build`** | — |
| **[6/6]** | Finalize `~/.bashrc` (source install, etc.) | — |

#### Gazebo installation (optional)

You do **not** need Gazebo. On the robot or when installing in Docker, press **Enter** or **`0`** — `GZ_VERSION` stays unset.

| Choice | When to use | Sets `GZ_VERSION` |
|--------|-------------|-------------------|
| **0** Skip *(default)* | Robot / Docker / no simulation | *(not set)* |
| **1** Classic | Desktop / VM that needs sim | `classic` |
| **2** Harmonic | `gz-sim` on Ubuntu 22.04 | `harmonic` |

Gazebo is heavy — skip unless you need [Gazebo](gazebo.md).

#### Robot model (`UGV_MODEL`)

| Model | Chassis | Notes |
|-------|---------|-------|
| **`ugv_rover`** | 6-wheel 4WD | UGV Rover series |
| **`ugv_beast`** | Tracked | UGV Beast series |
| **`rasp_rover`** | 4WD | RaspRover series |

#### LiDAR model (`LDLIDAR_MODEL`)

| Model | Baud rate |
|-------|-----------|
| `ld06`, `ld19` | 230400 |
| `stl27l` | 921600 |

Must match the LiDAR shipped with your ROS2 Kit.

---

## Post-installation verification

Open a **new terminal** or run:

```bash
source ~/.bashrc
echo $UGV_MODEL $LDLIDAR_MODEL $GZ_VERSION
```

`GZ_VERSION` empty is normal if you skipped Gazebo.

Full variable reference: [index — environment variables](index.md#product-names-vs-environment-variables).

**Next:** [Robot Description](description.md) or [Hardware Driver](bringup.md).

---

## Rebuild after code changes

```bash
cd /home/ws/ugv_ws
bash build_common.sh
```

Interactive package picker (no model / Gazebo prompts). When finished, `source install/setup.bash` in that terminal; other open terminals need `source ~/.bashrc` or a new shell.

Or rebuild everything:

```bash
cd /home/ws/ugv_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Change model or LiDAR

Edit `~/.bashrc`:

```bash
export UGV_MODEL=ugv_rover
export LDLIDAR_MODEL=ld19
source ~/.bashrc
```

No need to re-run full `build_first.sh` for **`UGV_MODEL`** or **`LDLIDAR_MODEL`** alone.

**Simulation (`GZ_VERSION`):** adding Gazebo or switching backend requires installing the matching apt packages — re-run `build_first.sh` and select the Gazebo version you need. See [Gazebo](gazebo.md).

---

## Related Tutorials

| Chapter | What it adds |
|---------|----------------|
| [ROS2 Basics](ros2_basics.md) | Terms used during install |
| [UGV Basics](ugv_basics.md) | Dual-controller layout, kit types |
| [Hardware Driver](bringup.md) | Boot the robot after install |
| [Gazebo](gazebo.md) | Simulation (only if Gazebo was installed) |
