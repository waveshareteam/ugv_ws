import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

# ---------------------------------------------------------------------------
# bringup_full.launch.py — full UGV stack
#
# Starts:
#   - ugv_bringup      ESP32 serial, motors, IMU, odometry, voltage
#   - ldlidar          LD19 laser scan  (port: LDLIDAR_PORT env var, default /dev/ttyAMA1)
#   - rf2o             laser odometry
#   - ugv_base_node    odom → TF
#   - ugv_description  robot URDF / TF tree
#   - usb_cam          USB webcam  (/dev/video0 → /image_raw)
#   - oak_d_lite       OAK-D Lite  (/oak/rgb/*, /oak/stereo/*)
#
# Launch arguments:
#   use_webcam    true/false  (default true)
#   use_oak       true/false  (default true)
#   use_rviz      true/false  (default false)
#
# Example:
#   ros2 launch ugv_bringup bringup_full.launch.py
#   ros2 launch ugv_bringup bringup_full.launch.py use_oak:=false
# ---------------------------------------------------------------------------

def generate_launch_description():

    use_webcam_arg = DeclareLaunchArgument(
        'use_webcam', default_value='true',
        description='Launch USB webcam node'
    )
    use_oak_arg = DeclareLaunchArgument(
        'use_oak', default_value='true',
        description='Launch OAK-D Lite node'
    )
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='false',
        description='Launch RViz2'
    )

    # --- Base platform + lidar + odometry + TF ---
    bringup_lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ugv_bringup'),
                         'launch', 'bringup_lidar.launch.py')
        ),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items()
    )

    # --- USB webcam ---
    from launch.conditions import IfCondition
    webcam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ugv_vision'),
                         'launch', 'camera.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('use_webcam'))
    )

    # --- OAK-D Lite ---
    oak = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ugv_vision'),
                         'launch', 'oak_d_lite.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('use_oak'))
    )

    return LaunchDescription([
        use_webcam_arg,
        use_oak_arg,
        use_rviz_arg,
        bringup_lidar,
        webcam,
        oak,
    ])
