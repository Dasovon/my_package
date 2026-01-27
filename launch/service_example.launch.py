from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='demo_nodes_cpp',
            executable='add_two_ints_server',
            name='add_two_ints_server',
            output='screen'
        ),
    ])