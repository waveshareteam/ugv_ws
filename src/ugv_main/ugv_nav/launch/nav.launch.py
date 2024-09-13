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

# Function to get the localplan config file based on the launch configuration
def get_localplan_config_file(context):
    # Get the use_localplan and use_localization launch configurations
    use_localplan = context.launch_configurations['use_localplan']
    use_localization = context.launch_configurations['use_localization']

    # Get the package share directory for ugv_nav
    ugv_nav_dir = get_package_share_directory('ugv_nav')

    # Get the paths to the different localplan config files
    amcl_teb_param_path = os.path.join(ugv_nav_dir, 'param', 'amcl_teb.yaml')
    amcl_dwa_param_path = os.path.join(ugv_nav_dir, 'param', 'amcl_dwa.yaml')
    emcl_teb_param_path = os.path.join(ugv_nav_dir, 'param', 'emcl_teb.yaml')
    emcl_dwa_param_path = os.path.join(ugv_nav_dir, 'param', 'emcl_dwa.yaml')

    # Create a dictionary to map the localplan and localization configurations to their respective config files
    config_map = {
        ('amcl', 'teb'): amcl_teb_param_path,
        ('amcl', 'dwa'): amcl_dwa_param_path,
        ('emcl', 'teb'): emcl_teb_param_path,
        ('emcl', 'dwa'): emcl_dwa_param_path
    }

    # Return the config file based on the launch configurations
    return config_map.get((use_localization, use_localplan), amcl_teb_param_path)

# Function to set up the launch description
def launch_setup(context, *args, **kwargs):

    # Get the use_localplan launch configuration
    use_localplan = context.launch_configurations['use_localplan']
    
    # Get the localplan config file
    param_file = get_localplan_config_file(context)
    
    # Get the package share directories for ugv_nav, nav2_bringup, and emcl2
    ugv_nav_dir = get_package_share_directory('ugv_nav')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    emcl_dir = get_package_share_directory('emcl2')

    # Get the map yaml path
    map_yaml_path = LaunchConfiguration('map', default=os.path.join(ugv_nav_dir, 'maps', 'map.yaml'))
    # Get the emcl param file
    emcl_param_file = os.path.join(emcl_dir, 'config', 'emcl2_quick_start.param.yaml')                        
    # Include the bringup_lidar launch description
    bringup_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ugv_bringup'), 'launch', 'bringup_lidar.launch.py')),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
            'rviz_config': 'nav_2d', 
        }.items()
    )

    # Include the nav2_bringup_amcl launch description if use_localization is amcl
    nav2_bringup_amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml_path,
            'params_file': param_file,
        }.items(),
        condition=LaunchConfigurationEquals('use_localization', 'amcl')
    )

    # Include the nav2_bringup_emcl launch description if use_localization is emcl
    nav2_bringup_emcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ugv_nav_dir, 'launch/nav_bringup', 'nav2_bringup.launch.py')),
        launch_arguments={
            'map': map_yaml_path,
            'params_file': param_file
        }.items(),
        condition=LaunchConfigurationEquals('use_localization', 'emcl')
    )
    
    # Include the emcl launch description if use_localization is emcl
    emcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(emcl_dir, 'launch', 'emcl2.launch.py')),
        launch_arguments={
            'params_file': emcl_param_file,
        }.items(),
        condition=LaunchConfigurationEquals('use_localization', 'emcl')
    )
    
    # Include the nav2_bringup_cartographer launch description if use_localization is cartographer
    nav2_bringup_cartographer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ugv_nav'), 'launch/nav_bringup', 'bringup_launch_cartographer.launch.py')),
         launch_arguments={
            'params_file': os.path.join(get_package_share_directory('ugv_nav'), 'param', 'emcl_dwa.yaml')
        }.items(),
        condition=LaunchConfigurationEquals('use_localization', 'cartographer')
    )
    
    # Include the robot_pose_publisher launch description
    robot_pose_publisher_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('robot_pose_publisher'), 'launch'),
         '/robot_pose_publisher_launch.py'])
    ) 
    
    # Return the list of launch descriptions
    return [
        bringup_lidar_launch,
        nav2_bringup_amcl_launch,
        nav2_bringup_emcl_launch,
        emcl_launch,
        robot_pose_publisher_launch,
        nav2_bringup_cartographer_launch
    ]

# Function to generate the launch description
def generate_launch_description():
    # Return the launch description
    return LaunchDescription([
        DeclareLaunchArgument('use_localplan', default_value='teb', description='Choose which localplan to use: dwa,teb'),
        DeclareLaunchArgument('use_localization', default_value='amcl', description='Choose which use_localization to use: amcl,cartographer'),
        OpaqueFunction(function=launch_setup)
    ])

# Main function to run the launch description
if __name__ == '__main__':
    generate_launch_description()
