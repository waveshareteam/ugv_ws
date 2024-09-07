from ament_index_python.packages import get_package_share_path
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    ugv_bringup_dir = get_package_share_directory('ugv_bringup')
    rviz_config_dir = os.path.join(ugv_bringup_dir,'rviz','view_ugv_bringup.rviz')
        
    pub_odom_tf_arg = DeclareLaunchArgument('pub_odom_tf', default_value='true',
                                            description='Whether to publish the tf from the original odom to the base_footprint')                                         
     
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='false',
                                         description='Whether to launch RViz2')  

    rviz_config_arg = DeclareLaunchArgument('rviz_config', default_value='bringup',
                                         description='Choose which rviz configuration to use')  
                                                  
    robot_state_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ugv_description'), 'launch', 'display.launch.py')
        ),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
            'rviz_config': LaunchConfiguration('rviz_config'),
        }.items()
    ) 
                                   
    bringup_node = Node(
        package='ugv_bringup',
        executable='ugv_bringup',
    )

    driver_node = Node(
        package='ugv_bringup',
        executable='ugv_driver',
    )
    
    laser_bringup_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('ldlidar'), 'launch'),
         '/ldlidar.launch.py'])
    )

    rf2o_laser_odometry_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('rf2o_laser_odometry'), 'launch'),
         '/rf2o_laser_odometry.launch.py'])
    ) 
       
    base_node = Node(
        package='ugv_base_node',
        executable='base_node',
        parameters=[{'pub_odom_tf': LaunchConfiguration('pub_odom_tf')}]
    )

    return LaunchDescription([
        pub_odom_tf_arg,
        use_rviz_arg,
        rviz_config_arg,
        robot_state_launch,
        bringup_node,
        driver_node,
        laser_bringup_launch,
        rf2o_laser_odometry_launch,
        base_node
    ])

