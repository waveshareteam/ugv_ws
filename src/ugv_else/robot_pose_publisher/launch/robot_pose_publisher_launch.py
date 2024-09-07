from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="robot_pose_publisher", executable="robot_pose_publisher",
            name="robot_pose_publisher",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"use_sim_time": True},
                {"is_stamped": True},
                {"map_frame": "map"},
                {"base_frame": "base_link"}
            ]
        )
    ])
