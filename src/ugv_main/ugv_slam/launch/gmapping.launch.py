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

    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='false',
                                     description='Whether to launch RViz2')
                                     
    bringup_lidar_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('ugv_bringup'), 'launch'),
         '/bringup_lidar.launch.py']),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
            'rviz_config': 'slam_2d',
        }.items()
    )
    
    gmapping_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('slam_gmapping'), 'launch'),
         '/mapping.launch.py'])
    )  

    robot_pose_publisher_node = Node(package="robot_pose_publisher", executable="robot_pose_publisher",
            name="robot_pose_publisher",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"use_sim_time": False},
                {"is_stamped": True},
                {"map_frame": "map"},
                {"base_frame": "base_footprint"}
            ]
    ) 
        
    return LaunchDescription([
        use_rviz_arg,
        bringup_lidar_launch, 
        gmapping_launch,
        robot_pose_publisher_node
    ])
