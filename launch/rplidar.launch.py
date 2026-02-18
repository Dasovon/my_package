#!/usr/bin/env python3
# Copyright (c) 2026 Ryan
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Get the config file path
    config = os.path.join(
        get_package_share_directory('my_robot_bringup'),
        'config',
        'rplidar.yaml',
    )

    return LaunchDescription([
        # RPLIDAR Node
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            parameters=[config],
            output='screen',
            respawn=True,
            respawn_delay=3.0,
        ),
    ])
