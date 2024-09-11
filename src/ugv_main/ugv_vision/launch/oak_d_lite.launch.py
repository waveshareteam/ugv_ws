import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    name = LaunchConfiguration("name").perform(context)
    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    params_file = LaunchConfiguration("params_file")
    
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(depthai_prefix, "launch", "camera.launch.py")
            ),
            launch_arguments={"name": name, "parent_frame": '3d_camera_link',"params_file": params_file}.items(),
        ),
        LoadComposableNodes(
            condition=IfCondition(LaunchConfiguration("rectify_rgb")),
            target_container=name + "_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="image_proc",
                    plugin="image_proc::RectifyNode",
                    name="rectify_color_node",
                    remappings=[
                        ("image", name + "/rgb/image_raw"),
                        ("camera_info", name + "/rgb/camera_info"),
                        ("image_rect", name + "/rgb/image_rect"),
                        ("image_rect/compressed", name + "/rgb/image_rect/compressed"),
                        (
                            "image_rect/compressedDepth",
                            name + "/rgb/image_rect/compressedDepth",
                        ),
                        ("image_rect/theora", name + "/rgb/image_rect/theora"),
                    ],
                )
            ],
        )
    ]


def generate_launch_description():
    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    declared_arguments = [
        DeclareLaunchArgument("name", default_value="oak"),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(get_package_share_directory("ugv_vision"), "config", "oak_d_lite.yaml"),
        ),
        DeclareLaunchArgument("rectify_rgb", default_value="True"),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
