import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Get the config file path
    config = os.path.join(
        get_package_share_directory('my_robot_bringup'),
        'config',
        'rplidar.yaml'
    )

    return LaunchDescription([
        # RPLIDAR Node
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            parameters=[config],
            output='screen'
        ),
    ])