# apriltag_ros ROS2 Node

Inspired from https://github.com/christianrauch/apriltag_ros.git and fuse features from ros1 version.

Apriltag_ros is ROS2 wrapper for apriltag detection. It take sensor_msgs/Image as input, and return pose and position in tf2 format.

You can specify number in tag family to filter detection of output, and set frame name for each of them. Please set the size of each tag correctly to make sure publish tf is accurate.

## Quickstart

Starting with a working ROS installation:

```
mkdir -p ~/apriltag_ros2_ws/src                # Make a new workspace 
cd ~/apriltag_ros2_ws/src                      # Navigate to the source space
git clone https://github.com/AprilRobotics/apriltag.git                  # Clone Apriltag library
git clone https://github.com/Adlink-ROS/apriltag_ros.git -b foxy-devel   # Clone Apriltag ROS wrapper
cd ..               
rosdep install --from-paths src --ignore-src -r -y  # Install any missing packages
colcon build --symlink-install                      # Build all packages in the workspace
```

## Usage

Launch tag detection node, and specify the camera namespace and topic name. Image topic will remap to "camera_name/image_topic".

```
ros2 launch apriltag_ros tag_realsense.launch.py camera_name:=/camera/color image_topic:=image_raw
```

### Custom parameter

Modify configuration file in /apriltag_ros/apriltag_ros/cfg/

You can also change config file to load in launch file.
