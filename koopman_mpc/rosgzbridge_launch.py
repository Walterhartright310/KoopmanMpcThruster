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
            '/horizontal_pos@std_msgs/msg/Float64@gz.msgs.Double',
            ],
        ),
         
        Node(
            package='ros2gzbridge',
            executable='odometry_subscriber',
            name='odometry_subscriber',
            parameters=[
                {'p_gain': 0.13},
                {'d_gain': -100.0},
                {'depth': 2.0},
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            output='screen',
        ),
        
        # Publish to the cmd_vel topic
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '--once','/tethys/propeller_cml', 'std_msgs/msg/Float64', 'data: 30'],
            output='screen',
        )
        
    ])



