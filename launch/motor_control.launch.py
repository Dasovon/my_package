#!/usr/bin/env python3
# Copyright (c) 2026 Ryan
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get config file
    config = os.path.join(
        get_package_share_directory('my_robot_bringup'),
        'config',
        'motor_controller.yaml',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'publish_odom_tf',
            default_value='true',
            description='Publish odom->base_footprint TF. Set false when using EKF.',
        ),

        Node(
            package='my_robot_bringup',
            executable='motor_controller.py',
            name='motor_controller',
            parameters=[config, {'publish_odom_tf': LaunchConfiguration('publish_odom_tf')}],
            output='screen',
        ),
    ])
