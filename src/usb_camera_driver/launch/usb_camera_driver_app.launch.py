"""
ROS 2 USB Camera Driver app launch file.

Roberto Masocco <robmasocco@gmail.com>
Lorenzo Bianchi <lnz.bnc@gmail.com>
Intelligent Systems Lab <isl.torvergata@gmail.com>

August 7, 2023
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Builds a LaunchDescription for the ROS 2 USB Camera Driver app"""
    ld = LaunchDescription()

    # Build config file path
    config = os.path.join(
        get_package_share_directory('usb_camera_driver'),
        'config',
        'config.yaml'
    )

    # Declare launch arguments
    nm = LaunchConfiguration('name')
    ns = LaunchConfiguration('namespace')
    cf = LaunchConfiguration('cf')
    nm_launch_arg = DeclareLaunchArgument(
        'name',
        default_value='usb_camera_driver'
    )
    ns_launch_arg = DeclareLaunchArgument(
        'namespace',
        default_value='usb_camera'
    )
    cf_launch_arg = DeclareLaunchArgument(
        'cf',
        default_value=config
    )
    ld.add_action(nm_launch_arg)
    ld.add_action(ns_launch_arg)
    ld.add_action(cf_launch_arg)

    # Create node launch description
    node = Node(
        package='usb_camera_driver',
        executable='usb_camera_app',
        name=nm,
        namespace=ns,
        exec_name='usb_camera_app',
        emulate_tty=True,
        output='both',
        log_cmd=True,
        parameters=[cf]
    )

    ld.add_action(node)

    return ld
