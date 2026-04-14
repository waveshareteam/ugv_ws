FROM dudulrx0601/ugv_rpi_ros_humble:ugv_rpi_ros_humble 

# Install build deps
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    git \
    cmake \
    && rm -rf /var/lib/apt/lists/*

# Default: copy local source (always reflects current working tree).
# To build straight from GitHub instead, pass --build-arg USE_GIT=1:
#   docker build --build-arg USE_GIT=1 -f docker/Dockerfile .
# Run from the repo root so the build context contains ./src.
SHELL ["/bin/bash", "-c"]
WORKDIR /home/ws/ugv_ws

RUN git clone -b ros2-humble-develop https://github.com/tranzmatt/ugv_ws.git /home/ws/ugv_ws && \
    find /etc/apt/sources.list.d/ -name "*ros*" -delete && \
    find /usr/share/keyrings/ -name "*ros*" -delete && \
    apt-get update && apt-get install -y curl gnupg2 ca-certificates lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
      http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
      > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && \
    . /opt/ros/humble/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y \
        --skip-keys "gazebo cmake_modules roslaunch"

RUN cd /home/ws/ugv_ws && \
    chmod +x *.sh && \
    ./build_first.sh && \
    ./build_common.sh && \
    ./build_apriltag.sh && \
    rm -rf /var/lib/apt/lists/*

# Set up interactive shell environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo 'eval "$(register-python-argcomplete ros2)"' >> ~/.bashrc && \
    echo 'eval "$(register-python-argcomplete colcon)"' >> ~/.bashrc && \
    echo "source /home/ws/ugv_ws/install/setup.bash" >> ~/.bashrc

ENTRYPOINT ["/bin/bash", "-c"]
CMD ["bash"]
