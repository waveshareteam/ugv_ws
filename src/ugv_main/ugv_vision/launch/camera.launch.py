import os

# Import the necessary modules from the ament_index_python package
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node,ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

# Define a function to generate the launch description
def generate_launch_description():

    # Get the package directory for the ugv_vision package
    pkg_dir = get_package_share_directory('ugv_vision')
    # Get the path to the params.yaml file
    param_file = os.path.join(pkg_dir, 'config', 'params.yaml') 
    
    # Create a node for the usb_cam package
    camera_node = Node(
            package='usb_cam', 
            executable='usb_cam_node_exe', 
            name='usb_cam',
            parameters= [param_file]
            #remappings=camera.remappings
    )

    # Declare a launch argument for the namespace
    arg_namespace = DeclareLaunchArgument(
        name='namespace', default_value='',
        description=('namespace for all components loaded')
    )

    # Create a list of composable nodes
    composable_nodes = [
        ComposableNode(
            package='image_proc',
            plugin='image_proc::RectifyNode',
            name='rectify_color_node',
            namespace=LaunchConfiguration('namespace'),
            # Remap subscribers and publishers
            remappings=[
                ('image', 'image_raw'),
                ('image_rect', 'image_rect')
            ],
        )
    ]

    # Declare a launch argument for the container
    arg_container = DeclareLaunchArgument(
        name='container', default_value='',
        description=(
            'Name of an existing node container to load launched nodes into. '
            'If unset, a new container will be created.'
        )
    )

    # If an existing container is not provided, start a container and load nodes into it
    image_processing_container = ComposableNodeContainer(
        condition=LaunchConfigurationEquals('container', ''),
        name='image_proc_container',
        namespace=LaunchConfiguration('namespace'),
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,
        output='screen'
    )

    # If an existing container name is provided, load composable nodes into it
    # This will block until a container with the provided name is available and nodes are loaded
    load_composable_nodes = LoadComposableNodes(
        condition=LaunchConfigurationNotEquals('container', ''),
        composable_node_descriptions=composable_nodes,
        target_container=LaunchConfiguration('container'),
    )
                
    # Return the launch description
    return LaunchDescription([
        camera_node,
        arg_namespace,
        arg_container,
        image_processing_container,
        load_composable_nodes,
    ])
