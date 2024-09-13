# Import necessary modules
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

# Define a function to generate the launch description
def generate_launch_description():
    # Include the launch description for the ugv_web_app
    ugv_web_app_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('vizanti_server'), 'launch'),
         '/vizanti_server.launch.py']),        
        launch_arguments={
            'host': LaunchConfiguration('host'),
        }.items()
    )
        
    # Return the launch description
    return LaunchDescription([
        ugv_web_app_launch
    ])