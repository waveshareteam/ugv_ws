from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    ugv_web_app_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('vizanti_server'), 'launch'),
         '/vizanti_server.launch.py'])
    )
        
    return LaunchDescription([
        ugv_web_app_launch
    ])
