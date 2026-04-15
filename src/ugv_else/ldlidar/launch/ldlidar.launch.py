#!/usr/bin/env python3
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
import os

# ---------------------------------------------------------------------------
# Configuration via environment variables
# ---------------------------------------------------------------------------
# LDLIDAR_MODEL (default: ld19)
#   Which lidar model to launch: ld06 | ld19 | stl27l
#
# LDLIDAR_PORT (default: /dev/ttyAMA1)
#   Serial port the lidar is wired to.  The device name depends on which
#   UART overlay is active in /boot/firmware/config.txt on the host:
#
#     Overlay                          Device
#     -------------------------------- ---------------
#     uart0-pi5  (GPIO14/15, default)  /dev/ttyAMA0   ← used by ESP32 driver board
#     uart1-pi5,pins_32_33             /dev/ttyAMA1
#     uart2-pi5,pins_34_35             /dev/ttyAMA2
#     uart3-pi5,pins_4_5               /dev/ttyAMA3
#     (USB-serial adapter)             /dev/ttyUSB0 or /dev/ttyACM0
#
#   Example /boot/firmware/config.txt for lidar on GPIO32/33:
#     enable_uart=1
#     dtoverlay=uart1-pi5,pins_32_33
#
#   To verify which device appeared after boot:
#     ls -la /dev/ttyAMA*
#     dmesg | grep tty
#
#   Set the port when starting the container, e.g.:
#     docker run -e LDLIDAR_PORT=/dev/ttyAMA1 ...
#   or export it before calling ros2 launch:
#     export LDLIDAR_PORT=/dev/ttyAMA1
#
#   You can also override at launch time without an env var:
#     ros2 launch ldlidar ldlidar.launch.py port_name:=/dev/ttyUSB0
# ---------------------------------------------------------------------------

def generate_launch_description():

    LDLIDAR_MODEL = os.environ.get('LDLIDAR_MODEL', 'ld19')
    LDLIDAR_PORT  = os.environ.get('LDLIDAR_PORT',  '/dev/ttyAMA1')
    ldlidar_launch_file = LDLIDAR_MODEL + '.launch.py'

    laser_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ldlidar'), 'launch/'),
             ldlidar_launch_file]),
        launch_arguments={'port_name': LDLIDAR_PORT}.items()
    )

    ld = LaunchDescription()

    ld.add_action(laser_bringup_launch)

    return ld
