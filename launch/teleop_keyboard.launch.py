#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Declare launch arguments
    linear_speed_arg = DeclareLaunchArgument(
        'linear_speed',
        default_value='0.2',
        description='Linear speed in m/s'
    )

    angular_speed_arg = DeclareLaunchArgument(
        'angular_speed',
        default_value='0.5',
        description='Angular speed in rad/s'
    )

    return LaunchDescription([
        linear_speed_arg,
        angular_speed_arg,

        # Teleop Keyboard Node
        Node(
            package='my_robot_bringup',
            executable='teleop_keyboard.py',
            name='teleop_keyboard',
            parameters=[{
                'linear_speed': LaunchConfiguration('linear_speed'),
                'angular_speed': LaunchConfiguration('angular_speed'),
            }],
            output='screen',
            prefix='xterm -e',
        ),
    ])
