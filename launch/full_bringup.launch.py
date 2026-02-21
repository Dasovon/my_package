#!/usr/bin/env python3
# Copyright (c) 2026 Ryan
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
"""
Full robot bringup launch file.

Launches all core robot nodes:
- Robot State Publisher (URDF/TF)
- Motor Controller (odometry + cmd_vel)
- RPLIDAR (laser scan)
- BNO055 IMU
- robot_localization EKF (fused odometry)

Usage:
    ros2 launch my_robot_bringup full_bringup.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('my_robot_bringup')

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock',
    )

    return LaunchDescription([
        use_sim_time_arg,

        # Robot State Publisher (URDF + TF tree)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'rsp.launch.py')
            ),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
        ),

        # Motor Controller (odometry + cmd_vel, TF disabled - EKF publishes TF)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'motor_control.launch.py'),
            ),
            launch_arguments={
                'publish_odom_tf': 'true',
            }.items(),
        ),

        # RPLIDAR A1 (laser scan)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'rplidar.launch.py'),
            ),
        ),

        # BNO055 IMU
        Node(
            package='bno055',
            executable='bno055',
            name='bno055',
            parameters=[os.path.join(pkg_dir, 'config', 'bno055.yaml')],
        ),

        # Static transform: base_link -> imu_link
        # Adjust xyz if IMU is not at robot center
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
        ),

        # EKF - fuses wheel odometry + IMU, publishes odom -> base_footprint TF
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            parameters=[os.path.join(pkg_dir, 'config', 'ekf.yaml')],
        ),

        # Lidar watchdog - stops motor when /scan has no subscribers (saves battery)
        Node(
            package='my_robot_bringup',
            executable='lidar_watchdog.py',
            name='lidar_watchdog',
            output='screen',
        ),
    ])
