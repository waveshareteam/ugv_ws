import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
 
# Function to generate launch description
def generate_launch_description():
    # Get the use_sim_time launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # Get the ugv_cartographer prefix
    ugv_cartographer_prefix = get_package_share_directory('ugv_cartographer')
    # Get the cartographer config directory
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                  ugv_cartographer_prefix, 'config'))
    # Get the configuration basename
    configuration_basename = LaunchConfiguration('configuration_basename',
                                                 default='localization_2d.lua')
 
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    ugv_cartographer_prefix = get_package_share_directory('ugv_cartographer')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                  ugv_cartographer_prefix, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename',
                                                 default='localization_2d.lua')
    # Get the resolution launch configuration
    resolution = LaunchConfiguration('resolution', default='0.05')
    # Get the publish period in seconds launch configuration
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
 
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    # Path to the pbstream file
    pbstream_path = "/home/ws/ugv_ws/src/ugv_main/ugv_gazebo/maps/map.pbstream"
 
    # Return the launch description
    return LaunchDescription([
        # Declare the cartographer config directory launch argument
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to config file to load'),
        # Declare the configuration basename launch argument
        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer'),
        # Declare the use_sim_time launch argument
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

    pbstream_path = "/home/ws/ugv_ws/src/ugv_main/ugv_gazebo/maps/map.pbstream"
    return LaunchDescription([
        # Create a node to run the cartographer node
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename,
                       '-load_state_filename', pbstream_path]),
        # Declare the resolution launch argument
        DeclareLaunchArgument(
            'cartographer_config_dir',
            default_value=cartographer_config_dir,
            description='Full path to config file to load'),
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'),
 
        # Declare the publish period in seconds launch argument
        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'),
 
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename,
                       '-load_state_filename', pbstream_path]),
        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'),
        # Include the occupancy grid launch description
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/occupancy_grid.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time, 'resolution': resolution,
                              'publish_period_sec': publish_period_sec}.items(),
        ),
    ])
 
        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'),
 
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/occupancy_grid.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time, 'resolution': resolution,
                              'publish_period_sec': publish_period_sec}.items(),
        ),
    ])
