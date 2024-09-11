#!/bin/bash

# 切换到地图文件所在目录
cd /home/ws/ugv_ws/src/ugv_main/ugv_nav/maps

# 保存地图
ros2 run nav2_map_server map_saver_cli -f ./map

# 调用服务，将地图状态保存为 pbstream 格式
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '/home/ws/ugv_ws/src/ugv_main/ugv_nav/maps/map.pbstream'}"

# 返回到初始目录
cd -

