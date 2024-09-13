import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition

# Function to get the appropriate RViz configuration file based on the input parameter
def get_rviz_config_file(context):
    rviz_config = context.launch_configurations['rviz_config']

    # Get the package directories for the UGV project
    ugv_description_dir = get_package_share_directory('ugv_description')
    ugv_bringup_dir = get_package_share_directory('ugv_bringup')
    ugv_slam_dir = get_package_share_directory('ugv_slam')
    ugv_nav_dir = get_package_share_directory('ugv_nav')

    # Define paths for different RViz configuration files
    rviz_description_config = os.path.join(ugv_description_dir, 'rviz', 'view_description.rviz')
    rviz_bringup_config = os.path.join(ugv_bringup_dir, 'rviz', 'view_bringup.rviz')
    rviz_slam_2d_config = os.path.join(ugv_slam_dir, 'rviz', 'view_slam_2d.rviz')
    rviz_slam_3d_config = os.path.join(ugv_slam_dir, 'rviz', 'view_slam_3d.rviz')
    rviz_nav_2d_config = os.path.join(ugv_nav_dir, 'rviz', 'view_nav_2d.rviz')
    rviz_nav_3d_config = os.path.join(ugv_nav_dir, 'rviz', 'view_nav_3d.rviz')

    # Map configuration options to corresponding RViz files
    config_map = {
        'description': rviz_description_config,
        'bringup': rviz_bringup_config,
        'slam_2d': rviz_slam_2d_config,
        'slam_3d': rviz_slam_3d_config,
        'nav_2d': rviz_nav_2d_config,
        'nav_3d': rviz_nav_3d_config
    }

    # Return the corresponding RViz configuration file, defaulting to 'description'
    return config_map.get(rviz_config, rviz_description_config)

# Function to set up and launch ROS 2 nodes based on the given context
def launch_setup(context, *args, **kwargs):

    rviz_config = context.launch_configurations['rviz_config']
    UGV_MODEL = os.environ['UGV_MODEL']
    urdf_file_name = UGV_MODEL + '.urdf'
    urdf_model_path = os.path.join(
        get_package_share_directory('ugv_description'),
        'urdf', 
        urdf_file_name)      
        
    # Determine whether to use the joint_state_publisher_gui based on the rviz configuration
    use_joint_state_publisher_gui = 'true' if rviz_config == 'description' else context.launch_configurations.get('use_joint_state_publisher_gui', 'false')

    # Define the robot_state_publisher node to publish the robot's URDF model
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='ugv',
        arguments=[urdf_model_path]
    )

    # Define the joint_state_publisher_gui node if the GUI is enabled
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        namespace='ugv',
        name='joint_state_publisher_gui',
        arguments=[urdf_model_path],
        condition=IfCondition(use_joint_state_publisher_gui)
    )

    # Define the joint_state_publisher node to publish joint states when the GUI is disabled
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace='ugv',
        name='joint_state_publisher',
        arguments=[urdf_model_path],
        condition=UnlessCondition(use_joint_state_publisher_gui)
    )

    # Get the appropriate RViz configuration file
    rviz_config_file = get_rviz_config_file(context)

    # Define the RViz2 node to launch RViz if enabled
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    # Return a list of nodes to launch
    return [
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        joint_state_publisher_node,
        rviz2_node
    ]

# Function to generate the launch description with configurable arguments
def generate_launch_description():
    return LaunchDescription([
        # Argument to specify whether to use the joint_state_publisher GUI
        DeclareLaunchArgument('use_joint_state_publisher_gui', default_value='false', description='Whether to launch joint_state_publisher GUI'),
        # Argument to specify whether to use RViz
        DeclareLaunchArgument('use_rviz', default_value='false', description='Whether to launch RViz2'),
        # Argument to specify which RViz configuration to use
        DeclareLaunchArgument('rviz_config', default_value='description', description='Choose which rviz configuration to use: description, bringup, slam_2d, slam_3d, nav_2d, nav_3d'),
        # Opaque function to execute the setup
        OpaqueFunction(function=launch_setup)
    ])

# Main entry point for launching the description
if __name__ == '__main__':
    generate_launch_description()
