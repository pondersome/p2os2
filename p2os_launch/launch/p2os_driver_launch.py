from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='p2os_driver',
            executable='p2os_driver',
            name='p2os_driver',
            parameters=[
                {'use_sonar': False}
            ],
            arguments=['--ros-args', '--log-level', 'INFO']
        ),            
    ])
