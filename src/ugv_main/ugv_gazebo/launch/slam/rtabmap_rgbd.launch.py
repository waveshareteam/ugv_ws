from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    queue_size = LaunchConfiguration('queue_size')
    qos = LaunchConfiguration('qos')
    localization = LaunchConfiguration('localization')
    
    # Launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_queue_size = DeclareLaunchArgument(
        'queue_size', default_value='20',
        description='Queue size'
    )
    
    declare_qos = DeclareLaunchArgument(
        'qos', default_value='2',
        description='QoS used for input sensor topics'
    )
        
    declare_localization = DeclareLaunchArgument(
        'localization', default_value='false',
        description='Launch in localization mode.'
    )

    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='false',
                                     description='Whether to launch RViz2')  

    # Parameters for the rtabmap_slam node
    parameters={
          'frame_id':'base_footprint',
          'use_sim_time':use_sim_time,
          'queue_size':queue_size,
          'subscribe_depth':True,
          'subscribe_rgb': True,
          'subscribe_scan': True,
          'use_action_for_goal':True,
          'qos_image':qos,
          'qos_imu':qos,
          'Reg/Force3DoF':'true',
          'Optimizer/GravitySigma':'0' # Disable imu constraints (we are already in 2D)
    }

    # Remappings for the topics
    remappings=[
          ('rgb/image', '/camera/image_raw'),
          ('rgb/camera_info', '/camera/camera_info'),
          ('depth/image', '/camera/depth/image_raw')]

    # SLAM mode:
    rtabmap_slam_node_slam = Node(
        condition=UnlessCondition(localization),
        package='rtabmap_slam', executable='rtabmap', output='screen',
        parameters=[parameters],
        remappings=remappings,
        arguments=['-d']
    )  # This will delete the previous database (~/.ros/rtabmap.db)
            
    # Localization mode:
    rtabmap_slam_node_localization = Node(
        condition=IfCondition(localization),
        package='rtabmap_slam', executable='rtabmap', output='screen',
        parameters=[
            parameters,
            {'Mem/IncrementalMemory': 'False',
             'Mem/InitWMWithAllNodes': 'True'}
        ],
        remappings=remappings
    )
    
    # Get the path to the ugv_gazebo package
    ugv_slam_dir = get_package_share_directory('ugv_gazebo')
    # Get the path to the rviz config file
    rviz_slam_3d_config = os.path.join(ugv_slam_dir, 'rviz', 'view_slam_3d.rviz')
    
    # Launch rviz2 node if use_rviz is true
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_slam_3d_config],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )
    
    # Launch rtabmap_viz node if use_rviz is false
    rtabmap_viz_node = Node(
        package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        parameters=[parameters],
        remappings=remappings,
        condition=UnlessCondition(LaunchConfiguration('use_rviz'))
    )

    # Launch robot_pose_publisher node
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
                      
    return LaunchDescription([
        declare_use_sim_time,
        declare_queue_size,
        declare_qos,
        declare_localization,
        use_rviz_arg,
        rtabmap_slam_node_slam,
        rtabmap_slam_node_localization,
        rviz2_node,
        rtabmap_viz_node,
        robot_pose_publisher_node
    ])
