#!/bin/bash

ros2 pkg list | grep -q turtlebot3_gazebo || { echo install turtlebot3_gazebo ; exit 1; }
ros2 pkg list | grep -q nav2_bringup || { echo install nav2_bringup ; exit 1; }

TURTLEBOT3_MODEL=burger ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py &
sleep 3
ros2 launch nav2_bringup rviz_launch.py &
sleep 1
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true &
sleep 5
ros2 launch emcl2 emcl2.launch.py params_file:=$(ros2 pkg prefix --share emcl2)/config/emcl2_quick_start.param.yaml map:=$(ros2 pkg prefix --share nav2_bringup)/maps/turtlebot3_world.yaml use_sim_time:=true
