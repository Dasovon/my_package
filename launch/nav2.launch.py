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
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg_dir = get_package_share_directory('my_robot_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    map_file = os.path.join(pkg_dir, 'maps', 'my_map.yaml')
    params_file = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    # Run RSP locally on the dev machine so static TFs (base_footprint→base_link,
    # base_link→laser_frame) are published here. Without this, Fast DDS cross-machine
    # transient_local delivery is unreliable and AMCL silently drops all scans.
    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={'use_sim_time': 'false'}.items(),
    )

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

    # Delay nav2_bringup by 3s so RSP's /tf_static is published before
    # AMCL processes its first scan. Without this delay AMCL silently fails
    # the TF lookup on the first scan and gets stuck.
    return LaunchDescription([
        rsp_launch,
        TimerAction(period=3.0, actions=[nav2_launch]),
    ])
