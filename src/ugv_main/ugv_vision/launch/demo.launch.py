import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to launch RViz2',
    )

    use_bringup_arg = DeclareLaunchArgument(
        'use_bringup',
        default_value='true',
        description=(
            'Include bringup_lidar. Set false when ugv_roarm_bringup '
            'or another base stack is already running.'
        ),
    )

    exe_arg = DeclareLaunchArgument(
        'exe',
        description='Vision demo executable (e.g. color_ball_track, cam_oak_webrtc)',
    )

    track_id_arg = DeclareLaunchArgument(
        'track_id',
        default_value='12',
        description='COCO class id for oak_object_track (ignored by other exes)',
    )

    package = 'ugv_vision'

    # OAK nodes open the camera via DepthAI; skip v4l2 USB pipeline for those exes.
    use_usb_camera = PythonExpression(["'oak' not in '", LaunchConfiguration('exe'), "'"])

    # Hardware only — no Gazebo path for vision demos.
    bringup_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ugv_bringup'), 'launch'),
             '/bringup_lidar.launch.py']),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
            'rviz_config': 'slam_2d',
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_bringup')),
    )

    cam_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(package), 'launch', 'camera.launch.py',
            ),
        ),
        condition=IfCondition(use_usb_camera),
    )

    node = Node(
        package=package,
        executable=LaunchConfiguration('exe'),
        output='screen',
        parameters=[{'track_id': LaunchConfiguration('track_id')}],
    )

    return LaunchDescription([
        use_rviz_arg,
        use_bringup_arg,
        exe_arg,
        track_id_arg,
        bringup_lidar_launch,
        cam_bringup_launch,
        node,
    ])
