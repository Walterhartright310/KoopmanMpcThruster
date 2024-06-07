import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
import os

def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation (Gazebo) clock if true'),
                              
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
            '/model/equipped_tethys/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/tethys/propeller_cml@std_msgs/msg/Float64@gz.msgs.Double',
            ],
        ),
         
        Node(
            package='thruster_mpc_controller',
            executable='mpc_controller',
           
            output='screen',
        )
        
    ])



