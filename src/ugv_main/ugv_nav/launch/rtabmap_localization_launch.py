from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')
    localization = LaunchConfiguration('localization')
    config_rviz = os.path.join(
        get_package_share_directory('rtabmap_launch'), 'launch', 'config', 'rgbd.rviz'
    )

    parameters = {
            "frame_id": 'base_footprint',
            'queue_size': 20,
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

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument(
            'qos', default_value='2',
            description='QoS used for input sensor topics'),
            
        DeclareLaunchArgument(
            'localization', default_value='true',
            description='Launch in localization mode.'),
        
        DeclareLaunchArgument('rviz_cfg', default_value=config_rviz, 
                              description='Configuration path of rviz2.'),
        
        DeclareLaunchArgument('rviz', default_value='true', description='Launch RVIZ (optional).'),

        IncludeLaunchDescription(PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ugv_vision'), 'launch'),
                     '/oak_d_lite.launch.py']
        )),

        # Nodes to launch
        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            parameters=[{'approx_sync':True, 'approx_sync_max_interval':0.01, 'use_sim_time':use_sim_time, 'qos':qos}],
            remappings=remappings),

        # SLAM Mode:
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=[parameters],
            remappings=remappings,
            arguments=['-d']),
            
        # Localization mode:
        Node(
            condition=IfCondition(localization),
            package='rtabmap_slam', executable='rtabmap', 
            parameters=[parameters,
              {'Mem/IncrementalMemory':'False',
               'Mem/InitWMWithAllNodes':'True'}],
            remappings=remappings),

    ])
