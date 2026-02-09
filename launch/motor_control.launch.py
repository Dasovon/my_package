#!/usr/bin/env python3
# Copyright (c) 2026 Ryan
# SPDX-License-Identifier: MIT

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Get config file
    config = os.path.join(
        get_package_share_directory('my_robot_bringup'),
        'config',
        'motor_controller.yaml',
    )

    return LaunchDescription([
        # Motor Controller Node
        Node(
            package='my_robot_bringup',
            executable='motor_controller.py',
            name='motor_controller',
            parameters=[config],
            output='screen',
        ),
    ])
