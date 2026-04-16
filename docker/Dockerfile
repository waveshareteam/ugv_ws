FROM dudulrx0601/ugv_rpi_ros_humble:ugv_rpi_ros_humble

SHELL ["/bin/bash", "-c"]
WORKDIR /home/ws/ugv_ws

# Install build deps and refresh ROS + Gazebo apt repos
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep git cmake \
    curl gnupg2 ca-certificates lsb-release && \
    find /etc/apt/sources.list.d/ -name "*ros*" -delete && \
    find /usr/share/keyrings/ -name "*ros*" -delete && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
      http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
      > /etc/apt/sources.list.d/ros2.list && \
    curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
      https://packages.osrfoundation.org/gazebo/ubuntu-stable $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
      > /etc/apt/sources.list.d/gazebo-stable.list && \
    rm -rf /var/lib/apt/lists/*

# Default: copy local source (always reflects current working tree).
# To build straight from GitHub instead, pass --build-arg USE_GIT=1:
#   docker build --build-arg USE_GIT=1 -f docker/Dockerfile .
# Run from the repo root so the build context contains ./src.
ARG USE_GIT=0

RUN if [ "$USE_GIT" = "1" ]; then \
        git clone -b ros2-humble-develop \
            https://github.com/tranzmatt/ugv_ws.git /tmp/ugv_ws_git && \
        cp -r /tmp/ugv_ws_git/. /home/ws/ugv_ws/ && \
        rm -rf /tmp/ugv_ws_git; \
    fi

COPY . /tmp/ugv_ws_ctx/
RUN if [ "$USE_GIT" = "0" ]; then \
        cp -r /tmp/ugv_ws_ctx/. /home/ws/ugv_ws/; \
    fi && \
    rm -rf /tmp/ugv_ws_ctx

RUN apt-get update && \
    . /opt/ros/humble/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    chmod +x build_apriltag.sh build_first.sh build_common.sh && \
    ./build_apriltag.sh && \
    ./build_first.sh && \
    rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/bin/bash", "-c"]
CMD ["bash"]
