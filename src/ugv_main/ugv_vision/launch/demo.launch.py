import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import AndCondition, IfCondition, UnlessCondition
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

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock',
    )

    use_bringup_arg = DeclareLaunchArgument(
        'use_bringup',
        default_value='true',
        description=(
            'Include ugv_bringup / ugv_gazebo bringup. '
            'Set false when ugv_roarm_bringup or another base stack is already running.'
        ),
    )

    exe_arg = DeclareLaunchArgument(
        'exe',
        description='Vision demo executable (e.g. color_ball_track, cam_oak_webrtc)',
    )

    package = 'ugv_vision'

    # OAK nodes open the camera via DepthAI; skip v4l2 USB pipeline for those exes.
    use_usb_camera = PythonExpression(["'oak' not in '", LaunchConfiguration('exe'), "'"])

    bringup_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ugv_bringup'), 'launch'),
             '/bringup_lidar.launch.py']),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
            'rviz_config': 'slam_2d',
        }.items(),
        condition=AndCondition([
            IfCondition(LaunchConfiguration('use_bringup')),
            UnlessCondition(LaunchConfiguration('use_sim_time')),
        ]),
    )

    bringup_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ugv_gazebo'), 'launch'),
             '/bringup_gazebo.launch.py']),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
            'rviz_config': 'slam_2d',
        }.items(),
        condition=AndCondition([
            IfCondition(LaunchConfiguration('use_bringup')),
            IfCondition(LaunchConfiguration('use_sim_time')),
        ]),
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
        parameters=[],
    )

    return LaunchDescription([
        use_rviz_arg,
        use_sim_time_arg,
        use_bringup_arg,
        exe_arg,
        bringup_lidar_launch,
        bringup_gazebo_launch,
        cam_bringup_launch,
        node,
    ])
