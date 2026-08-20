import os
import xacro
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
    SetEnvironmentVariable,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

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

def launch_setup(context, *args, **kwargs):
    rviz_config = context.launch_configurations['rviz_config']
    share_dir = get_package_share_directory('ugv_description')    
    ugv_gazebo_dir = get_package_share_directory('ugv_gazebo')
    UGV_MODEL = os.environ['UGV_MODEL']
    GZ_VERSION = os.environ['GZ_VERSION']
    xacro_file_name = UGV_MODEL + '.xacro'
    xacro_file = os.path.join(
        share_dir,
        'urdf/bases', 
        xacro_file_name)    
        
    mappings = {
        "use_gazebo": "true",
        "GZ_VERSION": GZ_VERSION,
    }  
    robot_description_config = xacro.process_file(xacro_file, mappings=mappings)
    robot_urdf = robot_description_config.toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_urdf}
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": True}],
    )

    pt_joint_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["pt_joint_position_controller", "-c", "/controller_manager"],
        parameters=[{"use_sim_time": True}],
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
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    world = os.path.join(ugv_gazebo_dir,'worlds','ugv.world')
    models_path = os.path.join(ugv_gazebo_dir, 'models')

    gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=models_path + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
    )

    ign_model_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=models_path + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world,
        }.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )

    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', UGV_MODEL,
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    gzserver_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"), 
                "launch", 
                "gz_sim.launch.py"
            ]),
        ]),
        launch_arguments={
            'gz_args': ['-r -v4 ', world], 
            'on_exit_shutdown': 'true'
        }.items(),
    )

    ign_gazebo_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': os.path.join(ugv_gazebo_dir, 'config', 'ros_gz_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
    )

    spawn_robot_node = Node(
        package="ros_gz_sim",
        executable="create",
        name='create',
        arguments=[
            "-topic", "robot_description",
            "-name", 'ugv',
            "-robot_namespace", '',
            "-x", '0.0',
            "-y", '0.0',
            "-z", '0.0',
            '-allow_renaming', 'true'
        ],
        output="screen"
    )

    # Spawn finishes → joint_state_broadcaster → PT controller
    delay_joint_state_broadcaster_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=urdf_spawn_node if GZ_VERSION == 'classic' else spawn_robot_node,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    delay_pt_controller_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[pt_joint_position_controller_spawner],
        )
    )

    nodes = [        
        robot_state_publisher_node,
    ]

    if GZ_VERSION == 'classic':
        nodes.extend([        
            gazebo_model_path,
            gazebo_server,
            gazebo_client,
            urdf_spawn_node,
        ])

    elif GZ_VERSION == 'harmonic':
        nodes.extend([        
            ign_model_path,
            gzserver_node,
            ign_gazebo_bridge,
            spawn_robot_node,
        ])

    nodes.extend([        
        delay_joint_state_broadcaster_after_spawn,
        delay_pt_controller_after_joint_state_broadcaster,
        rviz2_node,
    ])

    return nodes

# Function to generate the launch description with configurable arguments
def generate_launch_description():
    return LaunchDescription([
        # Argument to specify whether to use RViz
        DeclareLaunchArgument('use_rviz', default_value='false', description='Whether to launch RViz2'),
        # Argument to specify which RViz configuration to use
        DeclareLaunchArgument('rviz_config', default_value='description', description='Choose which rviz configuration to use: description, bringup, slam_2d, slam_3d, nav_2d, nav_3d'),
        # Opaque function to execute the setup
        OpaqueFunction(function=launch_setup)
    ])
