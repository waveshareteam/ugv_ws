#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
  # RViZ2 settings
  rviz2_config = os.path.join(
      get_package_share_directory('ldlidar'),
      'rviz',
      'view.rviz'
  )

  rviz2_node = Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2_show_ld06',
      arguments=['-d',rviz2_config],
      output='screen'
  )

  ldlidar_node = Node(
      package='ldlidar',
      executable='ldlidar_node',
      name='LD06',
      output='screen',
      parameters=[
        {'product_name': 'LDLiDAR_LD06'},
        {'topic_name': 'scan'},
        {'frame_id': 'base_lidar_link'},
        {'port_name': '/dev/ttyACM0'},
        {'port_baudrate': 230400},
        {'laser_scan_dir': True},
        {'enable_angle_crop_func': True},
        {'angle_crop_min': 225.0},
        {'angle_crop_max': 315.0}
      ]
  )
  # base_link to base_laser tf node
  base_footprint_to_laser_tf_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_footprint_to_base_laser_ld06',
    arguments=['0','0','0','0','0','0','base_footprint','base_lidar_link']
  )

  # Define LaunchDescription variable
  ld = LaunchDescription()

  ld.add_action(ldlidar_node)
  ld.add_action(base_footprint_to_laser_tf_node)
  ld.add_action(rviz2_node)

  return ld
