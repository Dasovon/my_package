#!/usr/bin/env python3
# Copyright (c) 2026 Ryan
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
"""
Nav2 launch file for autonomous navigation.

Wraps nav2_bringup's bringup_launch.py with robot-specific settings.
Run this on the dev machine (not the Pi) to offload computation.

Usage:
    ros2 launch my_robot_bringup nav2.launch.py

Prerequisites:
    Pi:  ros2 launch my_robot_bringup full_bringup.launch.py
    Dev: sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_dir = get_package_share_directory('my_robot_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    map_file = os.path.join(pkg_dir, 'maps', 'my_map.yaml')
    params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'params_file': params_file,
            'use_sim_time': 'false',
        }.items(),
    )

    return LaunchDescription([nav2_launch])
