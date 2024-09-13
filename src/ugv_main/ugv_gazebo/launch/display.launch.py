import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define the package name
    package_name = 'ugv_gazebo'
    # Create a LaunchDescription object
    ld = LaunchDescription()
    # Find the package share directory
    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    # Define the path to the rviz configuration file
    rviz_config_path = os.path.join(pkg_share, 'rviz/display.rviz')
    
    # Get the UGV model from the environment variables
    UGV_MODEL = os.environ['UGV_MODEL']
    # Define the urdf file name
    urdf_file_name = UGV_MODEL + '.urdf'
    # Define the path to the urdf model
    urdf_model_path = os.path.join(
        get_package_share_directory('ugv_gazebo'),
        'urdf', 
        urdf_file_name)   
            
    # Create a robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_model_path]
        )

    # Create a joint state publisher node
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[urdf_model_path]
        )

    # Create a rviz2 node
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        )

    # Add the nodes to the launch description
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(rviz2_node)

    # Return the launch description
    return ld

