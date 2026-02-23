#!/usr/bin/env python3
"""Launch slam_toolbox with our robot's config.

Also runs RSP locally so static TFs (base_footprint→base_link, base_link→laser_frame)
are available on this machine. Without this, Fast DDS cross-machine transient_local
delivery is unreliable and slam_toolbox silently fails TF lookups.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    pkg_dir = get_package_share_directory('my_robot_bringup')

    slam_params = os.path.join(pkg_dir, 'config', 'slam.yaml')

    slam_launch = PathJoinSubstitution([
        FindPackageShare('slam_toolbox'),
        'launch', 'online_async_launch.py'
    ])

    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={'use_sim_time': 'false'}.items(),
    )

    return LaunchDescription([
        rsp_launch,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={'slam_params_file': slam_params}.items(),
        ),
    ])
