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

    # Declare launch argument for whether to launch RViz2
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='false',
                                     description='Whether to launch RViz2')      

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time',default_value='false',
                                    description='Use simulation/Gazebo clock')
                                            
    # Include launch description for bringing up the lidar
    bringup_lidar_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('ugv_bringup'), 'launch'),
         '/bringup_lidar.launch.py']),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
            'rviz_config': 'slam_2d',
        }.items(),
        condition=UnlessCondition(LaunchConfiguration('use_sim_time'))        
    )

    bringup_gazebo_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('ugv_gazebo'), 'launch'),
         '/bringup_gazebo.launch.py']),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
            'rviz_config': 'slam_2d',
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_sim_time'))        
    )

    # Include launch description for robot pose publisher
    robot_pose_publisher_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('robot_pose_publisher'), 'launch'),
         '/robot_pose_publisher_launch.py']),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    ) 
        
    # Include launch description for cartographer
    cartographer_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('cartographer'), 'launch'),
         '/mapping.launch.py']),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )   
    
    # Return launch description
    return LaunchDescription([
        use_rviz_arg,
        use_sim_time_arg,
        bringup_lidar_launch,
        bringup_gazebo_launch,
        robot_pose_publisher_launch,
        cartographer_launch
    ])
