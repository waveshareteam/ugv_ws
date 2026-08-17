# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml

def launch_setup(context, *args, **kwargs):

    namespace = LaunchConfiguration('namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    posegraph_file = LaunchConfiguration('posegraph_file')
    slam_toolbox_params_file = LaunchConfiguration('slam_toolbox_params_file')
    emcl2_params_file = LaunchConfiguration('emcl2_params_file')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    ugv_cartographer_prefix = get_package_share_directory('cartographer')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(
                                                  ugv_cartographer_prefix, 'config'))
    configuration_basename = LaunchConfiguration('configuration_basename',
                                                 default='localization_2d.lua')
 
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    pbstream_file = LaunchConfiguration('pbstream', default=os.path.join(get_package_share_directory('ugv_nav'), 'maps', 'map.pbstream'))
    
    use_localization = LaunchConfiguration('use_localization')
    use_localization_text = LaunchConfiguration('use_localization').perform(context)

    lifecycle_nodes = ['map_server']

    if use_localization_text == 'amcl':
        lifecycle_nodes = ['map_server', 'amcl']
    # else:
    #     lifecycle_nodes = ['map_server']

    rtabmap_parameters = {
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

    rtabmap_remappings = [
        ("rgb/image", "oak/rgb/preview/image_raw"),
        ("rgb/camera_info", "oak/rgb/preview/camera_info"),
        ("depth/image", "oak/stereo/image_raw"),
    ]
    use_sim_time_text = context.launch_configurations['use_sim_time']
    
    if use_sim_time_text == 'true':
        rtabmap_remappings=[
            ('rgb/image', '/oak/image_raw'),
            ('rgb/camera_info', '/oak/camera_info'),
            ('depth/image', '/oak/depth/image_raw')]

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    slam_toolbox_param_substitutions = {
        'use_sim_time': use_sim_time,
        'map_file_name': posegraph_file}

    slam_toolbox_configured_params = ParameterFile(
        RewrittenYaml(
            source_file=slam_toolbox_params_file,
            root_key=namespace,
            param_rewrites=slam_toolbox_param_substitutions,
            convert_types=True),
        allow_substs=True)

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
                condition=LaunchConfigurationNotEquals('use_localization', 'rtabmap')),
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
                condition=LaunchConfigurationEquals('use_localization', 'amcl')),
            Node(
                name='emcl2',
                package='emcl2',
                executable='emcl2_node',
                parameters=[emcl2_params_file, {'use_sim_time': use_sim_time}],
                output='screen',
                condition=LaunchConfigurationEquals('use_localization', 'emcl')),
            Node(
                package='cartographer_ros',
                executable='cartographer_node',
                name='cartographer_node',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=['-configuration_directory', cartographer_config_dir,
                        '-configuration_basename', configuration_basename,
                        '-load_state_filename', pbstream_file],
                condition=LaunchConfigurationEquals('use_localization', 'cartographer')),
            Node(
                package='slam_toolbox',
                executable='localization_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[slam_toolbox_configured_params],
                condition=LaunchConfigurationEquals('use_localization', 'slam_toolbox')),
            Node(
                package='rtabmap_slam', 
                executable='rtabmap', 
                name='rtabmap',
                output='screen',
                parameters=[rtabmap_parameters,
                {'Mem/IncrementalMemory':'False',
                'Mem/InitWMWithAllNodes':'True'}],
                remappings=rtabmap_remappings,
                condition=LaunchConfigurationEquals('use_localization', 'rtabmap')),                
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes}],
                condition=LaunchConfigurationNotEquals('use_localization', 'rtabmap')),              
        ]
    )

    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(use_composition),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_map_server',
                plugin='nav2_map_server::MapServer',
                name='map_server',
                parameters=[configured_params],
                remappings=remappings,
            ),
            ComposableNode(
                package='nav2_amcl',
                plugin='nav2_amcl::AmclNode',
                name='amcl',
                parameters=[configured_params],
                remappings=remappings,
            ),                   
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_localization',
                parameters=[{'use_sim_time': use_sim_time,
                             'autostart': autostart,
                             'node_names': lifecycle_nodes}],
            ),
        ],
    )

    use_composition_node = GroupAction(
        condition=IfCondition(PythonExpression(use_composition)),
        actions=[
            Node(
                name='emcl2',
                package='emcl2',
                executable='emcl2_node',
                parameters=[emcl2_params_file, {'use_sim_time': use_sim_time}],
                output='screen',
                condition=LaunchConfigurationEquals('use_localization', 'emcl')),
            Node(
                package='cartographer_ros',
                executable='cartographer_node',
                name='cartographer_node',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=['-configuration_directory', cartographer_config_dir,
                        '-configuration_basename', configuration_basename,
                        '-load_state_filename', pbstream_file],
                condition=LaunchConfigurationEquals('use_localization', 'cartographer')),   
            Node(
                package='slam_toolbox',
                executable='localization_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[slam_toolbox_configured_params],
                condition=LaunchConfigurationEquals('use_localization', 'slam_toolbox')),   
            Node(
                package='rtabmap_slam', 
                executable='rtabmap', 
                name='rtabmap',
                output='screen',
                parameters=[rtabmap_parameters,
                {'Mem/IncrementalMemory':'False',
                'Mem/InitWMWithAllNodes':'True'}],
                remappings=rtabmap_remappings,
                condition=LaunchConfigurationEquals('use_localization', 'rtabmap')),                                           
        ]
    )

    return [
        # Add the actions to launch all of the localiztion nodes
        load_nodes,
        use_composition_node,
        load_composable_nodes,
    ]

def generate_launch_description():

    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        description='Full path to map yaml file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_use_localization_cmd = DeclareLaunchArgument(
        'use_localization', default_value='amcl',
        description='Use localization method (amcl or emcl)')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='False',
        description='Use composed bringup if True')

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name', default_value='nav2_container',
        description='the name of conatiner that nodes will load in if use composition')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')

    return LaunchDescription([
        # Set environment variables
        stdout_linebuf_envvar,

        # Declare the launch options
        declare_namespace_cmd,
        declare_map_yaml_cmd,
        declare_use_sim_time_cmd,
        declare_params_file_cmd,
        declare_use_localization_cmd,
        declare_autostart_cmd,
        declare_use_composition_cmd,
        declare_container_name_cmd,
        declare_use_respawn_cmd,
        declare_log_level_cmd,

        OpaqueFunction(function=launch_setup)
    ])
