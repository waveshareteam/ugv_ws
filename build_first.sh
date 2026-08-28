#!/bin/bash
set -e

WS=/home/ws/ugv_ws
BASHRC=~/.bashrc
APT_PREFS=/etc/apt/preferences.d/block-gazebo-classic

add_if_not_exist () {
    grep -qxF "$1" "$BASHRC" || echo "$1" >> "$BASHRC"
}

set_bashrc_export () {
    local key="$1"
    local val="$2"
    sed -i "/^export ${key}=/d" "$BASHRC"
    echo "export ${key}=${val}" >> "$BASHRC"
    export "${key}=${val}"
}

load_bashrc_var () {
    local var="$1"
    local val
    val=$(grep -E "^export ${var}=" "$BASHRC" 2>/dev/null | tail -n1 | cut -d= -f2- | tr -d '"' | tr -d "'" || true)
    if [ -n "$val" ]; then
        export "$var=$val"
    fi
}

unhold_gz_related () {
    echo "🔓 Clearing apt holds that may block Gazebo installs..."
    local holds
    holds=$(apt-mark showhold 2>/dev/null || true)
    if [ -n "$holds" ]; then
        echo "$holds"
        # shellcheck disable=SC2086
        sudo apt-mark unhold $holds || true
    else
        echo "✔ No held packages"
    fi
}

remove_installed_pkgs () {
    local pattern="$1"
    local pkgs
    pkgs=$(dpkg -l 2>/dev/null | awk -v p="$pattern" '$1 ~ /^ii/ && $2 ~ p {print $2}' || true)
    if [ -n "$pkgs" ]; then
        echo "🧹 Removing packages matching /$pattern/:"
        echo "$pkgs"
        # shellcheck disable=SC2086
        sudo apt-get remove -y --purge $pkgs || true
    fi
}

purge_gazebo_classic () {
    echo "🧹 Purging Gazebo Classic..."
    remove_installed_pkgs '^gazebo'
    remove_installed_pkgs '^libgazebo'
    remove_installed_pkgs '^ros-humble-gazebo'
    sudo apt-get autoremove -y || true
}

purge_gazebo_fortress_ros_gz () {
    echo "🧹 Purging Fortress ros-gz / ignition gazebo6..."
    remove_installed_pkgs '^ros-humble-ros-gz-'
    remove_installed_pkgs '^ros-humble-ros-gz$'
    remove_installed_pkgs '^ros-humble-ign-ros2-control'
    remove_installed_pkgs '^libignition-gazebo6'
    remove_installed_pkgs '^libignition-gui6'
    remove_installed_pkgs '^libignition-rendering6'
    remove_installed_pkgs '^libignition-sensors6'
    sudo apt-get remove -y --purge ignition-tools ignition-transport11-cli || true
}

purge_gazebo_harmonic () {
    echo "🧹 Purging Gazebo Harmonic / gz..."
    remove_installed_pkgs '^ros-humble-ros-gzharmonic'
    remove_installed_pkgs '^ros-humble-gz-ros2-control'
    remove_installed_pkgs '^gz-harmonic'
    remove_installed_pkgs '^gz-tools'
    remove_installed_pkgs '^gz-sim'
    remove_installed_pkgs '^gz-plugin'
    remove_installed_pkgs '^gz-math'
    remove_installed_pkgs '^gz-common'
    remove_installed_pkgs '^gz-msgs'
    remove_installed_pkgs '^gz-transport'
    remove_installed_pkgs '^gz-rendering'
    remove_installed_pkgs '^gz-sensors'
    remove_installed_pkgs '^gz-physics'
    remove_installed_pkgs '^gz-gui'
    remove_installed_pkgs '^gz-fuel'
    remove_installed_pkgs '^gz-launch'
    remove_installed_pkgs '^libgz-'
    remove_installed_pkgs '^libsdformat14'
    purge_gazebo_fortress_ros_gz
    sudo apt-get autoremove -y || true
}

detect_installed_gz () {
    if dpkg -l 2>/dev/null | awk '
        $1 ~ /^ii/ && ($2 ~ /^gz-harmonic/ || $2 ~ /^ros-humble-ros-gzharmonic/) {found=1}
        END {exit !found}'; then
        echo "harmonic"
    elif dpkg -l 2>/dev/null | awk '
        $1 ~ /^ii/ && $2 ~ /^(gazebo|libgazebo11|ros-humble-gazebo-ros)(|-.*)$/ {found=1}
        END {exit !found}'; then
        echo "classic"
    else
        echo ""
    fi
}

block_gazebo_classic_apt () {
    echo "🔒 Blocking Classic gazebo packages in apt (Harmonic present)..."
    #cat > "$APT_PREFS" << 'EOF'
    sudo tee "$APT_PREFS" > /dev/null << 'EOF'
Package: gazebo
Pin: release *
Pin-Priority: -1

Package: gazebo-*
Pin: release *
Pin-Priority: -1

Package: libgazebo*
Pin: release *
Pin-Priority: -1

Package: ros-humble-gazebo*
Pin: release *
Pin-Priority: -1
EOF
}

allow_gazebo_classic_apt () {
    sudo rm -f "$APT_PREFS"
}

ensure_universe () {
    sudo apt-get install -y software-properties-common
    sudo add-apt-repository -y universe || true
    sudo apt-get update
}

install_gazebo_classic () {
    echo "✔ Installing Gazebo Classic (gazebo11)..."
    unhold_gz_related
    purge_gazebo_harmonic
    allow_gazebo_classic_apt
    ensure_universe
    sudo apt-get install -y \
      gazebo \
      gazebo-common \
      gazebo-plugin-base \
      ros-humble-gazebo-ros-pkgs \
      ros-humble-gazebo-ros2-control
    add_if_not_exist "source /usr/share/gazebo/setup.bash"
}

install_gazebo_harmonic () {
    echo "⚠️ Installing Gazebo Harmonic (gz-sim 8) for ROS 2 Humble..."
    echo "   https://gazebosim.org/docs/harmonic/ros_installation/"

    unhold_gz_related
    purge_gazebo_classic
    purge_gazebo_fortress_ros_gz
    block_gazebo_classic_apt

    sudo apt-get install -y curl lsb-release gnupg

    sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
      --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
https://packages.osrfoundation.org/gazebo/ubuntu-stable \
$(lsb_release -cs) main" \
      | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

    sudo apt-get update
    sudo apt-get install -y \
      gz-harmonic \
      ros-humble-ros-gzharmonic \
      ros-humble-gz-ros2-control
}

reload_bashrc () {
    # shellcheck disable=SC1090
    source "$BASHRC" 2>/dev/null || true

    # 非交互脚本里 bashrc 可能直接 return，再强制加载关键变量
    while IFS= read -r line; do
        eval "$line"
    done < <(grep -E '^export (GZ_VERSION|UGV_MODEL|LDLIDAR_MODEL|ROARM_MODEL|GRIPPER_TYPE)=' "$BASHRC" 2>/dev/null || true)

    echo "✔ Reloaded env from ~/.bashrc (GZ_VERSION=${GZ_VERSION:-none})"
}

echo "=============================="
echo "   UGV Build & Config Script"
echo "=============================="
echo

load_bashrc_var GZ_VERSION
DETECTED_GZ="$(detect_installed_gz)"

echo "[0/6] Preparing apt..."
unhold_gz_related
sudo apt-get update || true

# 只有「真的已装 Harmonic」才 block Classic；
# 不要仅因 bashrc 里残留 GZ_VERSION=harmonic 就 pin 住 classic
if [ "$DETECTED_GZ" = "harmonic" ]; then
    block_gazebo_classic_apt
    purge_gazebo_fortress_ros_gz
fi

sudo apt-get -f install -y || true
sudo apt-get autoremove -y || true
sudo apt-get update

# ---------- Basic system deps ----------
echo "[1/6] Installing basic dependencies..."
sudo apt-get install -y \
  python3-pip \
  python3-colcon-argcomplete \
  alsa-utils \
  screen \
  speech-dispatcher \
  speech-dispatcher-espeak \
  espeak-ng \
  gstreamer1.0-rtsp \
  software-properties-common \
  curl \
  lsb-release \
  gnupg \
  git-lfs

# ---------- Python deps ----------
echo
echo "⚠️  Python dependencies will be installed via pip"
echo "⚠️  It is STRONGLY recommended to use a virtualenv for AI/Vision"
read -p "Continue pip install requirements.txt? [y/N]: " PIP_CONFIRM
if [[ "$PIP_CONFIRM" =~ ^[Yy]$ ]]; then
    python3 -m pip install -r requirements.txt
else
    echo "⏭ Skipped pip install"
fi

# ---------- ROS 2 packages ----------
echo
echo "[2/6] Installing ROS 2 packages..."
if [ "$DETECTED_GZ" = "harmonic" ]; then
    echo "✔ Harmonic detected → keep Classic blocked"
    block_gazebo_classic_apt
fi

sudo apt-get install -y \
    ros-humble-cartographer-ros \
    ros-humble-cartographer-ros-msgs \
    ros-humble-cartographer-rviz \
    ros-humble-desktop \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-position-controllers \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-rosbridge-suite \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    ros-humble-rtabmap-ros \
    ros-humble-v4l2-camera \
    ros-humble-robot-localization \
    ros-humble-depthai-bridge \
    ros-humble-depthai-ros-driver \
    ros-humble-depthai-ros-msgs \
    ros-humble-depthai-descriptions \
    ros-humble-depthai-examples \
    ros-humble-depthai \
    ros-humble-depthai-filters \
    ros-humble-depthai-ros

# ---------- Gazebo ----------
echo
echo "=============================="
echo "     Gazebo Version Select"
echo "=============================="
echo "⚠️  Classic and Harmonic CANNOT coexist"
echo "⚠️  Humble: ros-humble-ros-gz* = Fortress; Harmonic needs ros-humble-ros-gzharmonic"
echo
DETECTED_GZ="$(detect_installed_gz)"
echo "Detected packages : ${DETECTED_GZ:-none}"
echo "GZ_VERSION in env : ${GZ_VERSION:-none}"
echo

GAZEBO_INSTALLED=false

if [ -n "$GZ_VERSION" ] && [ -n "$DETECTED_GZ" ] && [ "$GZ_VERSION" = "$DETECTED_GZ" ]; then
    echo "✔ Gazebo already installed (GZ_VERSION=$GZ_VERSION)"
    read -p "Reinstall / switch anyway? [y/N]: " REINSTALL_GZ
    if [[ "$REINSTALL_GZ" =~ ^[Yy]$ ]]; then
        echo "Select Gazebo version:"
        echo "  [1] Classic"
        echo "  [2] Harmonic"
        read -p "Your choice [1-2]: " GAZEBO_CHOICE
        case "$GAZEBO_CHOICE" in
          1) install_gazebo_classic; GZ_VERSION="classic"; GAZEBO_INSTALLED=true ;;
          2) install_gazebo_harmonic; GZ_VERSION="harmonic"; GAZEBO_INSTALLED=true ;;
          *) echo "⏭ Keep current"; GAZEBO_INSTALLED=true ;;
        esac
    else
        echo "⏭ Skip reinstallation"
        GAZEBO_INSTALLED=true
        if [ "$GZ_VERSION" = "harmonic" ]; then
            block_gazebo_classic_apt
            purge_gazebo_fortress_ros_gz
            unhold_gz_related
            sudo apt-get install -y ros-humble-ros-gzharmonic ros-humble-gz-ros2-control || true
        fi
    fi

elif [ -n "$DETECTED_GZ" ]; then
    echo "✔ Gazebo packages present: $DETECTED_GZ (env was '${GZ_VERSION:-none}')"
    read -p "Reuse detected '$DETECTED_GZ'? [Y/n]: " REUSE_GZ
    if [[ "$REUSE_GZ" =~ ^[Nn]$ ]]; then
        echo "Select Gazebo version:"
        echo "  [1] Classic"
        echo "  [2] Harmonic"
        read -p "Your choice [1-2]: " GAZEBO_CHOICE
        case "$GAZEBO_CHOICE" in
          1) install_gazebo_classic; GZ_VERSION="classic"; GAZEBO_INSTALLED=true ;;
          2) install_gazebo_harmonic; GZ_VERSION="harmonic"; GAZEBO_INSTALLED=true ;;
          *) GZ_VERSION="$DETECTED_GZ"; GAZEBO_INSTALLED=true ;;
        esac
    else
        GZ_VERSION="$DETECTED_GZ"
        GAZEBO_INSTALLED=true
        if [ "$GZ_VERSION" = "harmonic" ]; then
            block_gazebo_classic_apt
            purge_gazebo_fortress_ros_gz
            unhold_gz_related
            sudo apt-get install -y ros-humble-ros-gzharmonic ros-humble-gz-ros2-control || true
        fi
        echo "⏭ Reuse $GZ_VERSION"
    fi

else
    echo "Select Gazebo version to install:"
    echo "  [1] Gazebo Classic (gazebo11)"
    echo "  [2] Gazebo Harmonic (gz-sim 8)"
    echo "  [0] Skip Gazebo installation"
    echo
    read -p "Your choice [0-2]: " GAZEBO_CHOICE

    case "$GAZEBO_CHOICE" in
      1)
        install_gazebo_classic
        GZ_VERSION="classic"
        GAZEBO_INSTALLED=true
        ;;
      2)
        install_gazebo_harmonic
        GZ_VERSION="harmonic"
        GAZEBO_INSTALLED=true
        ;;
      0)
        echo "⏭ Skipped Gazebo installation"
        GZ_VERSION=""
        ;;
      *)
        echo "❌ Invalid choice, skipping Gazebo installation"
        GZ_VERSION=""
        ;;
    esac
fi

if [ "$GAZEBO_INSTALLED" = true ] && [ -n "$GZ_VERSION" ]; then
  set_bashrc_export "GZ_VERSION" "$GZ_VERSION"
fi

# ---------- ROS env ----------
echo
echo "[3/6] Configuring ROS environment..."
add_if_not_exist "source /opt/ros/humble/setup.bash"
# shellcheck disable=SC1090
source "$BASHRC" || true

# ---------- Model selection ----------
echo
echo "[4/6] Select UGV model:"
select UGV_MODEL in ugv_rover ugv_beast rasp_rover; do
    [ -n "$UGV_MODEL" ] && break
    echo "Invalid selection."
done

echo
echo "Select LiDAR model:"
select LDLIDAR_MODEL in ld19 ld06 stl27l; do
    [ -n "$LDLIDAR_MODEL" ] && break
    echo "Invalid selection."
done

echo
echo "Selected configuration:"
echo "  UGV_MODEL     = $UGV_MODEL"
echo "  LDLIDAR_MODEL = $LDLIDAR_MODEL"
echo "  Gazebo        = $GAZEBO_INSTALLED"
echo "  GZ_VERSION    = ${GZ_VERSION:-none}"

read -p "Save model selection to ~/.bashrc? [y/N]: " SAVE_ENV
if [[ "$SAVE_ENV" =~ ^[Yy]$ ]]; then
    set_bashrc_export "UGV_MODEL" "$UGV_MODEL"
    set_bashrc_export "LDLIDAR_MODEL" "$LDLIDAR_MODEL"
    echo "✔ Model selection saved to ~/.bashrc"
else
    export UGV_MODEL
    export LDLIDAR_MODEL
    echo "✔ Model selection exported for current shell only"
fi

# ---------- Git LFS model weights ----------
echo
echo "Pulling Git LFS model weights..."
cd "$WS" || exit 1
if ! command -v git-lfs >/dev/null 2>&1; then
    echo "❌ git-lfs is not installed"
    exit 1
fi
git lfs pull

# ---------- Build ----------
echo
echo "[5/6] Building workspace: $WS"
cd "$WS" || exit 1

COMMON_PKGS=(
  cartographer
  costmap_converter_msgs
  costmap_converter
  emcl2
  explore_lite
  explore_lite_msgs
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
)

if [ "$GZ_VERSION" = "harmonic" ]; then
  echo "✔ GZ_VERSION=harmonic → include gz_ros2_control (workspace overlay optional)"
  COMMON_PKGS+=(gz_ros2_control)
else
  echo "⏭ Skip gz_ros2_control (need harmonic)"
fi

colcon build \
  --packages-select "${COMMON_PKGS[@]}" \
  --symlink-install \
  --executor sequential

UGV_PKGS=(
  ugv_bringup
  ugv_chat_ai
  ugv_description
  ugv_nav
  ugv_slam
  ugv_tools
  ugv_vision
  ugv_voice
  ugv_web_app
)

if [ "$GAZEBO_INSTALLED" = true ]; then
  UGV_PKGS+=(ugv_gazebo)
else
  echo "⏭ Skip ugv_gazebo"
fi

colcon build \
  --packages-select "${UGV_PKGS[@]}" \
  --symlink-install \
  --executor sequential

# ---------- Final env ----------
echo
echo "[6/6] Finalizing environment..."
add_if_not_exist "source $WS/install/setup.bash"
add_if_not_exist "export PULSE_SERVER=unix:/run/user/1000/pulse/native"
add_if_not_exist "export XDG_RUNTIME_DIR=/run/user/1000"
add_if_not_exist "# ---- ROS 2 & colcon argcomplete ----"

add_if_not_exist 'if [ -f /usr/share/ros2cli/ros2cli-completion.bash ]; then source /usr/share/ros2cli/ros2cli-completion.bash; fi'
add_if_not_exist 'if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash; fi'

# shellcheck disable=SC1090
source "$BASHRC" || true

echo
echo "=============================="
echo "✔ Environment ready."
echo "✔ UGV_MODEL=$UGV_MODEL"
echo "✔ LDLIDAR_MODEL=$LDLIDAR_MODEL"
echo "✔ Gazebo installed: $GAZEBO_INSTALLED"
echo "✔ GZ_VERSION=${GZ_VERSION:-none}"
if [ "$GZ_VERSION" = "harmonic" ]; then
  echo "✔ Harmonic: gz-harmonic + ros-humble-ros-gzharmonic + ros-humble-gz-ros2-control"
  echo "  Launch tip: gz_sim.launch.py gz_version:=8"
elif [ "$GZ_VERSION" = "classic" ]; then
  echo "✔ Classic: gazebo11 + gazebo-ros-pkgs + gazebo-ros2-control"
fi
echo "=============================="

reload_bashrc
