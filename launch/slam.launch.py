#!/usr/bin/env python3
"""Launch slam_toolbox with our robot's config."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    slam_params = os.path.join(
        get_package_share_directory('my_robot_bringup'),
        'config', 'slam.yaml'
    )

    slam_launch = PathJoinSubstitution([
        FindPackageShare('slam_toolbox'),
        'launch', 'online_async_launch.py'
    ])

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={'slam_params_file': slam_params}.items(),
        ),
    ])
