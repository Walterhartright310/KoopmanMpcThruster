
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='thruster_mpc_controller',
            executable='mat_file_loader_node',
            name='mat_file_loader_node'
        )
    ])

