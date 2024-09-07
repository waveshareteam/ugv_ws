# Copyright (c) 2022 H-HChen
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

image_topic_ = LaunchConfiguration("image_topic", default="image_raw")
camera_name = LaunchConfiguration("camera_name", default="/camera/color")

image_topic = [camera_name, "/", image_topic_]
info_topic = [camera_name, "/camera_info"]
config = os.path.join(
    get_package_share_directory("apriltag_ros"), "cfg", "tags_36h11_filter.yaml"
)


def generate_launch_description():

    composable_node = ComposableNode(
        name="apriltag",
        package="apriltag_ros",
        plugin="AprilTagNode",
        parameters=[config],
        remappings=[("/image", image_topic), ("/camera_info", info_topic)],
    )

    container = ComposableNodeContainer(
        name="tag_container",
        namespace="apriltag",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[composable_node],
        output="screen",
    )

    return launch.LaunchDescription([container])
