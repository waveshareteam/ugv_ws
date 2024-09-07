import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = 'ugv_gazebo'
    ld = LaunchDescription()
    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    rviz_config_path = os.path.join(pkg_share, 'rviz/display.rviz')
    
    UGV_MODEL = os.environ['UGV_MODEL']
    urdf_file_name = UGV_MODEL + '.urdf'
    urdf_model_path = os.path.join(
        get_package_share_directory('ugv_gazebo'),
        'urdf', 
        urdf_file_name)  
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time',default_value='true', description='Use simulation (Gazebo) clock if true')
    rviz_config_path = os.path.join(get_package_share_directory('ugv_description'), 'config/display.rviz')   
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_model_path],
        parameters=[{
             'use_sim_time': use_sim_time
        }],
        )
                
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[urdf_model_path],
        parameters=[{
             'use_sim_time': use_sim_time
        }],        
        )       
                
    ld.add_action(use_sim_time_arg)     
    ld.add_action(robot_state_publisher_node) 
    ld.add_action(joint_state_publisher_node)
    
    return ld
