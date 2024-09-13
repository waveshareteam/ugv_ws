import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def get_localplan_config_file(context):
    use_localplan = context.launch_configurations['use_localplan']
    use_localization = context.launch_configurations['use_localization']

    ugv_nav_dir = get_package_share_directory('ugv_nav')

    amcl_teb_param_path = os.path.join(ugv_nav_dir, 'param', 'amcl_teb.yaml')
    amcl_dwa_param_path = os.path.join(ugv_nav_dir, 'param', 'amcl_dwa.yaml')
    emcl_teb_param_path = os.path.join(ugv_nav_dir, 'param', 'emcl_teb.yaml')
    emcl_dwa_param_path = os.path.join(ugv_nav_dir, 'param', 'emcl_dwa.yaml')

    config_map = {
        ('amcl', 'teb'): amcl_teb_param_path,
        ('amcl', 'dwa'): amcl_dwa_param_path,
        ('emcl', 'teb'): emcl_teb_param_path,
        ('emcl', 'dwa'): emcl_dwa_param_path
    }

    return config_map.get((use_localization, use_localplan), amcl_teb_param_path)

def launch_setup(context, *args, **kwargs):

    use_localplan = context.launch_configurations['use_localplan']
    
    param_file = get_localplan_config_file(context)
    
    ugv_nav_dir = get_package_share_directory('ugv_nav')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    emcl_dir = get_package_share_directory('emcl2')

    map_yaml_path = LaunchConfiguration('map', default=os.path.join(ugv_nav_dir, 'maps', 'map.yaml'))
    emcl_param_file = os.path.join(emcl_dir, 'config', 'emcl2_quick_start.param.yaml')                        
    bringup_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ugv_bringup'), 'launch', 'bringup_lidar.launch.py')),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
            'rviz_config': 'nav_2d', 
        }.items()
    )

    nav2_bringup_amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml_path,
            'params_file': param_file,
        }.items(),
        condition=LaunchConfigurationEquals('use_localization', 'amcl')
    )

    nav2_bringup_emcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ugv_nav_dir, 'launch/nav_bringup', 'nav2_bringup.launch.py')),
        launch_arguments={
            'map': map_yaml_path,
            'params_file': param_file
        }.items(),
        condition=LaunchConfigurationEquals('use_localization', 'emcl')
    )
    
    emcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(emcl_dir, 'launch', 'emcl2.launch.py')),
        launch_arguments={
            'params_file': emcl_param_file,
        }.items(),
        condition=LaunchConfigurationEquals('use_localization', 'emcl')
    )
    
    nav2_bringup_cartographer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ugv_nav'), 'launch/nav_bringup', 'bringup_launch_cartographer.launch.py')),
         launch_arguments={
            'params_file': os.path.join(get_package_share_directory('ugv_nav'), 'param', 'emcl_dwa.yaml')
        }.items(),
        condition=LaunchConfigurationEquals('use_localization', 'cartographer')
    )
    
    robot_pose_publisher_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('robot_pose_publisher'), 'launch'),
         '/robot_pose_publisher_launch.py'])
    ) 
    
    return [
        bringup_lidar_launch,
        nav2_bringup_amcl_launch,
        nav2_bringup_emcl_launch,
        emcl_launch,
        robot_pose_publisher_launch,
        nav2_bringup_cartographer_launch
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_localplan', default_value='teb', description='Choose which localplan to use: dwa,teb'),
        DeclareLaunchArgument('use_localization', default_value='amcl', description='Choose which use_localization to use: amcl,cartographer'),
        OpaqueFunction(function=launch_setup)
    ])

if __name__ == '__main__':
    generate_launch_description()
