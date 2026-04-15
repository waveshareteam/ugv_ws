docker run -it --rm \
  --name ugv_rpi_ros_humble \
  --network host \
  --ipc host \
  --privileged \
  -v /dev:/dev \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/ws/ugv_ws:/home/ws/ugv_ws \
  -e DISPLAY=$DISPLAY \
  dudulrx0601/ugv_rpi_ros_humble:ugv_rpi_ros_humble \
  bash

