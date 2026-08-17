from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument,TimerAction
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals

# Function to generate launch description
def generate_launch_description():

    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    ugv_slam_dir = get_package_share_directory('ugv_slam')

    use_sim_time_arg = DeclareLaunchArgument('use_sim_time',default_value='false',
                                    description='Use simulation/Gazebo clock')

    # Declare launch argument for whether to launch RViz2
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='false',
                                     description='Whether to launch RViz2')

    use_slam_arg = DeclareLaunchArgument('use_slam', default_value='sync',
                                     description='which slam to launch') 

    # Include launch description for bringup_lidar.launch.py
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
    
    # Include launch description for mapping.launch.py
    slam_toolbox_slam_async_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_toolbox_dir, 'launch','online_async_launch.py')),
        condition=LaunchConfigurationEquals('use_slam', 'async'),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'slam_params_file': os.path.join(ugv_slam_dir, 'config', 'slam_toolbox_online_async.yaml')
        }.items(),
    )

    # Include launch description for mapping.launch.py
    slam_toolbox_slam_sync_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_toolbox_dir, 'launch','online_sync_launch.py')),
        condition=LaunchConfigurationEquals('use_slam', 'sync'),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'slam_params_file': os.path.join(ugv_slam_dir, 'config', 'slam_toolbox_online_sync.yaml')
        }.items(),
    )  

    # Include launch description for robot_pose_publisher_launch.py
    robot_pose_publisher_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('robot_pose_publisher'), 'launch'),
         '/robot_pose_publisher_launch.py']),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    ) 
        
    # Return launch description
    return LaunchDescription([
        use_sim_time_arg,
        use_rviz_arg,
        use_slam_arg,
        bringup_lidar_launch, 
        bringup_gazebo_launch,
        robot_pose_publisher_launch,
        slam_toolbox_slam_async_launch,
        slam_toolbox_slam_sync_launch
    ])
