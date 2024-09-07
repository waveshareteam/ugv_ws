# emcl2_ros2: mcl with expansion resetting (version 2)

[![test](https://github.com/CIT-Autonomous-Robot-Lab/emcl2_ros2/actions/workflows/test.yml/badge.svg)](https://github.com/CIT-Autonomous-Robot-Lab/emcl2_ros2/actions/workflows/test.yml)

Emcl is an alternative Monte Carlo localization (MCL) package to amcl (http://wiki.ros.org/amcl). Differently from amcl, KLD-sampling and adaptive MCL are not implemented. Instead, the expansion resetting and other features are implemented[^1][^2].

This package is ROS 2 version of [ryuichiueda/emcl2](https://github.com/ryuichiueda/emcl2). 

## ROS 2 version 

* ROS 2 Humble Hawksbill

## quick start

### Install & Build
```
mkdir ros2_ws && cd ros2_ws
git clone git@github.com:CIT-Autonomous-Robot-Lab/emcl2_ros2.git ./src/emcl2_ros2
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
sudo apt install -y ros-$ROS_DISTRO-navigation2 ros-$ROS_DISTRO-nav2-bringup ros-$ROS_DISTRO-turtlebot3-gazebo
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### Try emcl2 in simulator

You may wait Gazebo to be initilalized. You can also use `./test/demo.bash`, in which the following procedure is written. 

```
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
ros2 launch nav2_bringup rviz_launch.py
ros2 launch emcl2 emcl2.launch.py params_file:=$(ros2 pkg prefix --share emcl2)/config/emcl2_quick_start.param.yaml map:=$(ros2 pkg prefix --share nav2_bringup)/maps/turtlebot3_world.yaml use_sim_time:=true
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
```

## demo movies 

[![](https://img.youtube.com/vi/dqS7KgGxwBs/0.jpg)](https://www.youtube.com/watch?v=dqS7KgGxwBs)

[![](https://img.youtube.com/vi/n9tzKY6ua_o/0.jpg)](https://www.youtube.com/watch?v=n9tzKY6ua_o)

## Nodes

### emcl2_node

This node calculates the alpha value with a different algorithm than `emcl_node` in [ryuichiueda/emcl](https://github.com/ryuichiueda/emcl). This node counts the particles that make lasers penetrate occupancy cells. Specifically, this node chooses some particles at a rate of `extraction_rate` and checks each of them with the following procedure:

* maps a set of laser scan on the occupancy grid map based on the pose of the particle
* judges the pose of the particle as wrong if all of lasers in a `range_threshold`[rad] range penatrate occupancy grids

If the rate of the wrong particles is greater than `alpha_threshold`, the node invokes a reset. 

This node also has a sensor resetting algorithm. When `sensor_reset` is true, a particle with laser penetration is dragged back from occupied cells. 

#### Subscribed Topics 

| Name        | Type                                    | Description                       | 
| :---------- | :-------------------------------------- | :-------------------------------- | 
| `map`         | [`nav_msgs/OccupancyGrid`](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html)                  | occupancy grid map                | 
| `scan`        | [`sensor_msgs/LaserScan`](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html)                   | laser scans                       | 
| `tf`          | [`tf/tfMessage`](http://docs.ros.org/en/noetic/api/tf/html/msg/tfMessage.html)                            | transforms                        | 
| `initialpose` | [`geometry_msgs/PoseWithCovarianceStamped`](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html) | pose of particles for replacement | 

#### Published Topics

| Name          | Type                                    | Description                                                                             | 
| ------------- | --------------------------------------- | --------------------------------------------------------------------------------------- | 
| `mcl_pose`      | [`geometry_msgs/PoseWithCovarianceStamped`](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html) | the mean pose of the particles with covariance                                          | 
| `particlecloud` | [`geometry_msgs/PoseArray`](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseArray.html)                 | poses of the particles                                                                  | 
| `tf`            | [`tf/tfMessage`](http://docs.ros.org/en/noetic/api/tf/html/msg/tfMessage.html)                            | the transform from odom (which can be remapped via the odom_frame_id parameter) to map | 
| `alpha`         | [`std_msgs/Float32`](http://docs.ros.org/en/noetic/api/std_msgs/html/msg/Float32.html)                        | marginal likelihood of particles after sensor update                                    | 

#### Parameters

| Name                          | Type    | Default    | Description                                                        |
|-------------------------------|---------|------------|--------------------------------------------------------------------|
| `odom_freq`                    | `int`     | 20 [Hz]    | frequency of odometry update                                       |
| `num_particles`                | `int`     | 500       | number of particles                                                |
| `odom_frame_id`                | `string`  | "odom"     | the frame for odometry                                             |
| `footprint_frame_id`           | `string`  | "base_footprint" | the frame of the localized robot's base                    |
| `base_frame_id`                | `string`  | "base_link" | the frame of the robot's base. It is used for calculating the position and orientation of the LiDAR |
| `global_frame_id`              | `string`  | "map"      | the frame for localization                                         |
| `initial_pose_x`               | `double`  | 0.0 [m]    | initial x coordinate of particles                                  |
| `initial_pose_y`               | `double`  | 0.0 [m]    | initial y coordinate of particles                                  |
| `initial_pose_a`               | `double`  | 0.0 [rad]  | initial yaw coordinate of particles                                |
| `odom_fw_dev_per_fw`           | `double`  | 0.19 [m/m] | standard deviation of forward motion noise by forward motion       |
| `odom_fw_dev_per_rot`          | `double`  | 0.0001 [m/rad] | standard deviation of forward motion noise by rotational motion |
| `odom_rot_dev_per_fw`          | `double`  | 0.13 [rad/m] | standard deviation of rotational motion noise by forward motion  |
| `odom_rot_dev_per_rot`         | `double`  | 0.2 [rad/rad] | standard deviation of rotational motion noise by rotational motion |
| `laser_likelihood_max_dist`    | `double`  | 0.2 meters | maximum distance to inflate occupied cells on the likelihood field map |
| `scan_increment`               | `int`     | 1          | increment number when beams are picked from their sequence         |
| `alpha_threshold`              | `double`  | 0.5        | threshold of the alpha value for expansion resetting               |
| `open_space_threshold`         | `double`  | 0.05       | threshold of the valid beam rate for expansion resetting           |
| `expansion_radius_position`    | `double`  | 0.1 [m]    | maximum change of the position on the xy-plane when the reset replaces a particle |
| `expansion_radius_orientation` | `double`  | 0.2 [rad]  | maximum change of the yaw angle when the reset replaces a particle |
| `extraction_rate`              | `double`  | 0.1        | rate of particles that are checked by the node                     |
| `range_threshold`              | `double`  | 0.3 [rad]  | threshold of the range of lasers                                   |
| `sensor_reset`                 | `bool`    | true       | flag for sensor resettings                                         |



The followings have never been implemented yet.

| Name              | Type   | Default          | Description                                               |
|-------------------|--------|------------------|-----------------------------------------------------------|
| `laser_min_range`  | `double` | 0.0 [m]          | threshold for discarding scans whose ranges are smaller than this value |
| `laser_max_range`  | `double` | 100000000.0 [m]  | threshold for discarding scans whose ranges are larger than this value  |

## To do

* Implement service
* Fix a bug that caused particles to survive

## citation

[^1]: R. Ueda: "[Syokai Kakuritsu Robotics (lecture note on probabilistic robotics)](https://www.amazon.co.jp/dp/B082SN3VTD)," Kodansya, 2019.

[^2]: R. Ueda, T. Arai, K. Sakamoto, T. Kikuchi, S. Kamiya: Expansion resetting for recovery from fatal error in Monte Carlo localization - comparison with sensor resetting methods, IEEE/RSJ IROS, pp.2481-2486, 2004. 
