#!/bin/bash
cd /home/ws/ugv_ws/src/ugv_main/ugv_gazebo/maps
ros2 run nav2_map_server map_saver_cli -f ./map
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '/home/ws/ugv_ws/src/ugv_main/ugv_gazebo/maps/map.pbstream'}"
cd -

