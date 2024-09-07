from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    return LaunchDescription([
        Node(
            package='slam_gmapping', 
            executable='slam_gmapping',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])
