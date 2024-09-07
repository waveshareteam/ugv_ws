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

    use_sim_time = LaunchConfiguration('use_sim_time')
    queue_size = LaunchConfiguration('queue_size')
    qos = LaunchConfiguration('qos')
    localization = LaunchConfiguration('localization')
    
    # Launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
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
                           
    parameters = {
            "frame_id": 'base_footprint',
            'queue_size': queue_size,
            "subscribe_rgb": True,
            "subscribe_depth": True,
            'subscribe_scan': True,
            "subscribe_odom_info": False,
            "approx_sync": True,
            "Rtabmap/DetectionRate": "3.5",
     }

    remappings = [
        ("rgb/image", "oak/rgb/image_rect"),
        ("rgb/camera_info", "oak/rgb/camera_info"),
        ("depth/image", "oak/stereo/image_raw"),
    ]
    
    bringup_lidar_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('ugv_bringup'), 'launch'),
         '/bringup_lidar.launch.py']),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
            'rviz_config': 'slam_3d',
        }.items()
    )
        
    bringup_oak_lite_launch = IncludeLaunchDescription( PythonLaunchDescriptionSource(
        [os.path.join(get_package_share_directory('ugv_slam'), 'launch'),
             '/oak_d_lite.launch.py']
        )
    )
    
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

    rtabmap_viz_node = Node(
        package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        parameters=[parameters],
        remappings=remappings,
        condition=UnlessCondition(LaunchConfiguration('use_rviz'))
    )

    robot_pose_publisher_node = Node(package="robot_pose_publisher", executable="robot_pose_publisher",
            name="robot_pose_publisher",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"use_sim_time": False},
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
        bringup_lidar_launch,
        bringup_oak_lite_launch,
        rtabmap_slam_node_slam,
        rtabmap_slam_node_localization,
        rtabmap_viz_node
        #robot_pose_publisher_node
    ])
