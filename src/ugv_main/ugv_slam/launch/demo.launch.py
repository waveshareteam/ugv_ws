import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Declare launch argument for whether to launch RViz2
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='false',
                                     description='Whether to launch RViz2')

    package = "ugv_slam"
    package_shared_path = get_package_share_directory(package)

    # Hardware only — no Gazebo path for LiDAR demos.
    bringup_lidar_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('ugv_bringup'), 'launch'),
         '/bringup_lidar.launch.py']),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
            'rviz_config': 'slam_2d',
        }.items(),
    )

    node = Node(
        package=package,
        executable=LaunchConfiguration("exe"),
        output="screen",
        parameters=[

        ],
    )

    arg = DeclareLaunchArgument(name="exe")
    return LaunchDescription([
        use_rviz_arg,
        bringup_lidar_launch,
        arg,
        node
    ])
