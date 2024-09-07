#!/bin/bash

# 切换到地图文件所在目录
cd /home/ws/ugv_ws/src/ugv_main/ugv_nav/maps

# 保存地图
ros2 run nav2_map_server map_saver_cli -f ./map

# 返回到初始目录
cd -

