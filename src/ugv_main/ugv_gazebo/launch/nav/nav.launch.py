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

# Function to get the localplan config file path based on the launch configuration
def get_localplan_config_file(context):
    use_localplan = context.launch_configurations['use_localplan']
    use_localization = context.launch_configurations['use_localization']

    ugv_nav_dir = get_package_share_directory('ugv_gazebo')

    # Get the path to the parameter files
    amcl_teb_param_path = os.path.join(ugv_nav_dir, 'param', 'amcl_teb.yaml')
    amcl_dwa_param_path = os.path.join(ugv_nav_dir, 'param', 'amcl_dwa.yaml')
    emcl_teb_param_path = os.path.join(ugv_nav_dir, 'param', 'emcl_teb.yaml')
    emcl_dwa_param_path = os.path.join(ugv_nav_dir, 'param', 'emcl_dwa.yaml')

    # Create a mapping of localization and localplan to parameter file paths
    config_map = {
        ('amcl', 'teb'): amcl_teb_param_path,
        ('amcl', 'dwa'): amcl_dwa_param_path,
        ('emcl', 'teb'): emcl_teb_param_path,
        ('emcl', 'dwa'): emcl_dwa_param_path
    }

    # Return the parameter file path based on the launch configuration
    return config_map.get((use_localization, use_localplan), amcl_teb_param_path)


# Function to set up the launch description
def launch_setup(context, *args, **kwargs):

    use_localplan = context.launch_configurations['use_localplan']
    use_localization = context.launch_configurations['use_localization']
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    param_file = get_localplan_config_file(context)
    
    ugv_gazebo_dir = get_package_share_directory('ugv_gazebo')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    emcl_dir = get_package_share_directory('emcl2')

    # Get the path to the map file
    map_yaml_path = LaunchConfiguration('map', default=os.path.join(ugv_gazebo_dir, 'maps', 'map.yaml'))
    rviz_slam_2d_config = os.path.join(ugv_gazebo_dir, 'rviz', 'view_nav_2d.rviz')
    emcl_param_file = os.path.join(emcl_dir, 'config', 'emcl2_quick_start.param.yaml')

    # Get the path to the AMCL parameter file
    nav2_bringup_amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml_path,
            'params_file': param_file,
            'use_sim_time': use_sim_time
        }.items(),
        condition=LaunchConfigurationEquals('use_localization', 'amcl')
     )

    # Get the path to the EMCL parameter file
    nav2_bringup_emcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ugv_gazebo_dir, 'launch/nav_bringup', 'nav2_bringup.launch.py')),
        launch_arguments={
            'map': map_yaml_path,
            'params_file': param_file,
            'use_sim_time': use_sim_time
        }.items(),
        condition=LaunchConfigurationEquals('use_localization', 'emcl')
    )
    
    emcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(emcl_dir, 'launch', 'emcl2.launch.py')),
        launch_arguments={
            'params_file': emcl_param_file,
            'use_sim_time': use_sim_time
        }.items(),
        condition=LaunchConfigurationEquals('use_localization', 'emcl')
    )
    
    # Get the path to the Cartographer parameter file
    nav2_bringup_cartographer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('ugv_gazebo'), 'launch/nav_bringup', 'bringup_launch_cartographer.launch.py')),
         launch_arguments={
            'params_file': os.path.join(get_package_share_directory('ugv_gazebo'), 'param', 'emcl_dwa.yaml'),
            'use_sim_time': use_sim_time
        }.items(),
        condition=LaunchConfigurationEquals('use_localization', 'cartographer')
    )
    
    # Create the RViz2 node
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_slam_2d_config]
    )

    # Create the robot pose publisher node
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
        nav2_bringup_amcl_launch,
        nav2_bringup_emcl_launch,
        emcl_launch,
        nav2_bringup_cartographer_launch,
        rviz2_node,
        robot_pose_publisher_node
    ]

# Function to generate the launch description
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_localplan', default_value='teb', description='Choose which localplan to use: dwa,teb'),
        DeclareLaunchArgument('use_localization', default_value='amcl', description='Choose which use_localization to use: amcl,emcl'),
        OpaqueFunction(function=launch_setup)
    ])

if __name__ == '__main__':
    generate_launch_description()
