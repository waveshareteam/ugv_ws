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

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true',
                                     description='Whether to launch gazebo')                          

    cartographer_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('cartographer'), 'launch'),
         '/mapping.launch.py']),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )
    
    ugv_slam_dir = get_package_share_directory('ugv_slam')
    rviz_slam_2d_config = os.path.join(ugv_slam_dir, 'rviz', 'view_slam_2d_gazebo.rviz')
    
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_slam_2d_config],
    )

    robot_pose_publisher_node = Node(package="robot_pose_publisher", executable="robot_pose_publisher",
            name="robot_pose_publisher",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"use_sim_time": True},
                {"is_stamped": True},
                {"map_frame": "map"},
                {"base_frame": "base_footprint"}
            ]
    ) 
                
    return LaunchDescription([
        use_sim_time_arg,
        cartographer_launch,
        rviz2_node,
        robot_pose_publisher_node
    ])
