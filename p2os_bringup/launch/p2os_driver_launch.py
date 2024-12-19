from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='p2os_driver',
            executable='p2os_driver',
            name='p2os_driver',
            remappings=[
                ('pose', 'odom')
            ],
            parameters=[
                {'use_sonar': False},
                {'port': '/dev/ttyUSB0'} #make sure this is the right port
            ],
            arguments=['--ros-args', '--log-level', 'INFO']
        ),            
    ])
