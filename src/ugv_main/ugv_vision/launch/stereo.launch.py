import os

# Import the necessary modules from the ament_index_python package
from ament_index_python.packages import get_package_share_directory
# Import the necessary modules from the launch package
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
# Import the necessary modules from the launch.conditions package
from launch.conditions import IfCondition
# Import the necessary modules from the launch.launch_description_sources package
from launch.launch_description_sources import PythonLaunchDescriptionSource
# Import the necessary modules from the launch.substitutions package
from launch.substitutions import LaunchConfiguration
# Import the necessary modules from the launch_ros.actions package
from launch_ros.actions import LoadComposableNodes, Node
# Import the necessary modules from the launch_ros.descriptions package
from launch_ros.descriptions import ComposableNode


# Define a function to set up the launch description
def launch_setup(context, *args, **kwargs):
    # Get the name of the launch configuration
    name = LaunchConfiguration("name").perform(context)
    # Get the prefix of the depthai_ros_driver package
    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    # Get the params_file of the launch configuration
    params_file = LaunchConfiguration("params_file")
    
    # Return a list of actions to be executed
    return [
        # Include the camera.launch.py launch file from the depthai_ros_driver package
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(depthai_prefix, "launch", "camera.launch.py")
            ),
            launch_arguments={
                "name": name, 
                "parent_frame": '3d_camera_link',
                "params_file": params_file,
                'use_rviz': 'False',
            }.items(),
        )
    ]


# Define a function to generate the launch description
def generate_launch_description():
    # Declare the launch arguments
    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak"),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(get_package_share_directory("ugv_vision"), "config", "stereo.yaml"),
        ),
        DeclareLaunchArgument("rectify_rgb", default_value="False"),
    ]

    # Return the launch description
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
