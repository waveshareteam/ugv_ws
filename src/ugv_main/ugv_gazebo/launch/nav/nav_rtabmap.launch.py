import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import OpaqueFunction
from launch.actions import DeclareLaunchArgument

def get_localplan_config_file(context):
    use_localplan = LaunchConfiguration('use_localplan').perform(context)

    ugv_nav_dir = get_package_share_directory('ugv_gazebo')

    teb_param_path = os.path.join(ugv_nav_dir, 'param', 'rtabmap_teb.yaml')
    dwa_param_path = os.path.join(ugv_nav_dir, 'param', 'rtabmap_dwa.yaml')

    config_map = {
        'teb': teb_param_path,
        'dwa': dwa_param_path
    }

    return config_map.get(use_localplan, teb_param_path)

def launch_setup(context, *args, **kwargs):

    param_file = get_localplan_config_file(context)
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    ugv_nav_dir = get_package_share_directory('ugv_gazebo')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # 定义启动参数
    map_yaml_path = LaunchConfiguration('map', default=os.path.join(ugv_nav_dir, 'maps', 'map.yaml'))
                                                         
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml_path,
            'params_file': param_file,
            'use_sim_time': use_sim_time
        }.items()
    )

    ugv_slam_dir = get_package_share_directory('ugv_gazebo')
    rviz_slam_3d_config = os.path.join(ugv_slam_dir, 'rviz', 'view_nav_3d.rviz')
    
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_slam_3d_config]
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
    
    return [
        nav2_bringup_launch,
        rviz2_node,
        robot_pose_publisher_node
    ]

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('use_localplan', default_value='dwa', description='Choose which localplan to use: dwa, teb'),
        OpaqueFunction(function=launch_setup)
    ])

if __name__ == '__main__':
    generate_launch_description()
