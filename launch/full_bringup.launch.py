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
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Serial port path for RPLIDAR reset command
_LIDAR_PORT = (
    '/dev/serial/by-id/'
    'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'
)

# Inline Python: flush serial buffers and send RPLIDAR firmware reset (0xA5 0x40).
# This resets the RPLIDAR MCU so the motor starts clean on every bringup,
# preventing the 80008002 / "Failed to set scan mode" crash loop.
_LIDAR_RESET_CMD = (
    'import serial, time; '
    f'p = serial.Serial("{_LIDAR_PORT}", 115200, timeout=1); '
    'p.reset_input_buffer(); p.reset_output_buffer(); '
    'p.write(bytes([0xA5, 0x40])); '
    'print("RPLIDAR firmware reset sent"); '
    'time.sleep(2); p.close()'
)


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

        # RPLIDAR A1: send firmware reset over serial before starting the node.
        # Clears bad state (motor stopped, mid-scan) left from the previous session.
        # The rplidar node is delayed 3s to give the firmware time to reinitialize.
        ExecuteProcess(
            cmd=['python3', '-c', _LIDAR_RESET_CMD],
            output='screen',
        ),
        TimerAction(
            period=3.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_dir, 'launch', 'rplidar.launch.py'),
                    ),
                ),
            ],
        ),

        # BNO055 IMU
        # Remap imu/imu -> imu/data to match ROS convention and EKF config.
        # The bno055 driver publishes to <prefix>imu (= imu/imu with prefix "imu/")
        # but robot_localization EKF expects /imu/data.
        Node(
            package='bno055',
            executable='bno055',
            name='bno055',
            parameters=[os.path.join(pkg_dir, 'config', 'bno055.yaml')],
            remappings=[('imu/imu', 'imu/data')],
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
