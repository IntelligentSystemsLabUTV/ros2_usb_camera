"""
USB camera calibrator app launch file.

Lorenzo Bianchi <lnz.bnc@gmail.com>
Roberto Masocco <robmasocco@gmail.com>
Intelligent Systems Lab <isl.torvergata@gmail.com>

July 10, 2022
"""

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Builds a LaunchDescription for the USB camera calibrator app"""
    ld = LaunchDescription()

    # Build config file path
    config = os.path.join(
        get_package_share_directory('ros2_usb_camera'),
        'config',
        'calibrator.yaml'
    )

    # Create node launch description
    node = Node(
        package='ros2_usb_camera',
        executable='calibrator',
        exec_name='calibrator_app',
        shell=True,
        emulate_tty=True,
        output='both',
        log_cmd=True,
        parameters=[config]
    )

    ld.add_action(node)

    return ld
