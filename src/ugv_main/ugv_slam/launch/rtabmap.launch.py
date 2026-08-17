import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode

def launch_setup(context, *args, **kwargs):

    use_viz = LaunchConfiguration("use_viz", default="true")
    use_rviz = LaunchConfiguration("use_rviz", default="false")
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    use_sim_time_text = context.launch_configurations['use_sim_time']
    # Hardware: EKF on unless visual/ICP odom will publish odom TF instead.
    use_odom_text = context.launch_configurations.get('use_odom', 'none')
    use_ekf = 'false' if use_odom_text in ('icp', 'rgbd') else 'true'

    parameters = [
        {
            "use_sim_time": use_sim_time,
            "frame_id": "base_footprint",
            "subscribe_rgb": True,
            "subscribe_depth": True,
            "subscribe_scan": True,
            "subscribe_odom_info": False,
            "approx_sync": True,
            "approx_sync_max_interval": 0.05, 
            "topic_queue_size": 30,
            "sync_queue_size": 30,
            "Rtabmap/DetectionRate": "1.0",
            "Rtabmap/LoopThr": "0.20",        
            # "Vis/MinInliers": "20",          
            # "RGBD/OptimizeMaxError": "3",     
            "Grid/Sensor": "0",                 
            "Grid/RangeMin": "0.2",
            "Grid/MinGroundHeight": "-0.2",
            "Grid/MaxGroundHeight": "0.0",
            "Grid/MaxObstacleHeight": "1.0",
            "Grid/NormalsSegmentation": "false",
        }
    ]

    remappings = [
        ("rgb/image", "oak/rgb/preview/image_raw"),
        ("rgb/camera_info", "oak/rgb/preview/camera_info"),
        ("depth/image", "oak/stereo/image_raw"),
    ]
    if use_sim_time_text == 'true':
        remappings=[
            ('rgb/image', '/oak/image_raw'),
            ('rgb/camera_info', '/oak/camera_info'),
            ('depth/image', '/oak/depth/image_raw')]

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [os.path.join(get_package_share_directory('ugv_bringup'), 'launch'),
                '/bringup_lidar.launch.py']),
                launch_arguments={
                    'use_rviz': use_rviz,
                    'rviz_config': 'slam_3d',
                    'use_ekf': use_ekf,
                }.items(),
                condition=UnlessCondition(use_sim_time)
        ),     
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ugv_vision'), 'launch'),
                '/oak_d_lite.launch.py']
            ),
            condition=UnlessCondition(use_sim_time)
        ), 
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ugv_gazebo'), 'launch'),
            '/bringup_gazebo.launch.py']),
            launch_arguments={
                'use_rviz': use_rviz,
                'rviz_config': 'slam_3d',
            }.items(),
            condition=IfCondition(LaunchConfiguration('use_sim_time'))        
        ),               
        # Real robot: load into oak_container from oak_d_lite.
        # Sim skips this block (no oak_container); use_odom is hardware-only.
        GroupAction(
            condition=UnlessCondition(use_sim_time),
            actions=[
                LoadComposableNodes(
                    target_container="oak_container",
                    condition=LaunchConfigurationEquals('use_odom', 'rgbd'),
                    composable_node_descriptions=[
                        ComposableNode(
                            package="rtabmap_odom",
                            plugin="rtabmap_odom::RGBDOdometry",
                            name="rgbd_odometry",
                            parameters=[
                            {
                                # "publish_tf": False,
                                "frame_id": 'base_footprint',
                                "use_sim_time": use_sim_time,
                            }
                            ],
                            remappings=remappings,
                        ),
                    ],
                ),
                LoadComposableNodes(
                    target_container="oak_container",
                    condition=LaunchConfigurationEquals('use_odom', 'icp'),
                    composable_node_descriptions=[
                        ComposableNode(
                            package="rtabmap_odom",
                            plugin="rtabmap_odom::ICPOdometry",
                            name="icp_odometry",
                            parameters=[
                            {
                                # "publish_tf": False,
                                "frame_id": 'base_footprint',
                                "use_sim_time": use_sim_time,
                            }
                            ],
                            remappings=remappings,
                        ),
                    ],
                ),
                LoadComposableNodes(
                    target_container="oak_container",
                    composable_node_descriptions=[
                        ComposableNode(
                            package="rtabmap_slam",
                            plugin="rtabmap_slam::CoreWrapper",
                            name="rtabmap",
                            parameters=parameters,
                            remappings=remappings,
                        ),
                    ],
                ),
            ],
        ),
        # Sim: standalone rtabmap only (use_odom icp/rgbd would fight Gazebo odom TF).
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            output='screen',
            parameters=parameters,
            remappings=remappings,
            condition=IfCondition(use_sim_time),
        ),
        Node(
            package="rtabmap_viz",
            executable="rtabmap_viz",
            output="screen",
            parameters=parameters,
            remappings=remappings,
            condition=IfCondition(use_viz),
        ),
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('use_viz', default_value='false',description='Whether to launch rtabmap_viz'),     
        DeclareLaunchArgument('use_rviz', default_value='false',description='Whether to launch RViz2'),     
        DeclareLaunchArgument('use_odom', default_value='none', description='Hardware odom: none=EKF, icp|rgbd=visual/ICP (disables EKF). Sim: leave none.'),
        DeclareLaunchArgument('use_sim_time',default_value='false',description='Use simulation/Gazebo clock')               
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )