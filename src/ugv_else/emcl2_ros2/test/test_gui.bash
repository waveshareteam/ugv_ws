#!/bin/bash -evx

ros2 launch emcl2 test.launch.xml &
sleep 50

### ESTIMATION RECOVERY TEST ###
# Publish initial pose
ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
pose:
  pose:
    position: {x: -2.5, y: 0.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
  covariance: [
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0]" | head -n 10

# Check if position is within a threshold
ros2 topic echo /mcl_pose --csv | 
head -n 1000 |
awk -F',' '{print $4" "$5}
     sqrt( ($4+2.0)^2 + ($5+0.5)^2 ) < 0.2 {printf "\033[42m%s\033[m\n", "ESTIMATION RECOVERY TEST OK";exit(0)}
     NR==1000{printf "\033[41m%s\033[m\n", "ESTIMATION RECOVERY TEST TIMEOUT";exit(1)}'

if [ "$?" -ne 0 ]; then
  ps aux | grep ros | grep -v grep | awk '{ print "kill -9", $2 }' | sh
  killall -9 gzclient gzserver rviz2
  exit 1
fi

ros2 service call /global_costmap/clear_entirely_global_costmap nav2_msgs/srv/ClearEntireCostmap request:\ {}\ 

sleep 10

### NAVIGATION TEST ###
# Publish nav2 goal
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "
pose:
  header: 
    stamp: 
      sec: 0
    frame_id: 'map'
  pose:
    position: { x: -0.5, y: -0.5, z: 0.0}
    orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0}" &

# Check if position is within a threshold
ros2 topic echo /mcl_pose --csv | 
head -n 1000 |
awk -F',' '{print $4" "$5}
     sqrt( ($4+0.5)^2 + ($5+0.5)^2 ) < 0.3 {printf "\033[42m%s\033[m\n", "NAVIGATION TEST OK";exit(0)}
     NR==1000{printf "\033[41m%s\033[m\n", "NAVIGATION TEST TIMEOUT";exit(1)}'

RESULT=$?

ps aux | grep ros | grep -v grep | awk '{ print "kill -9", $2 }' | sh
killall -9 gzclient gzserver rviz2

exit $RESULT
