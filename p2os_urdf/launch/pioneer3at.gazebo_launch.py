import os
import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Command to generate robot description using xacro
    robot_description_command = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([FindPackageShare('p2os_urdf'), 'defs', 'pioneer3at.xacro'])
    ])

    # Path to Gazebo empty world launch file
    gazebo_launch_file = PathJoinSubstitution([
        FindPackageShare('gazebo_ros'), 'launch', 'empty_world.launch.py'
    ])

    return LaunchDescription([
        # Node to publish robot state
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description_command,
                'publish_frequency': 30.0,
                'tf_prefix': ''
            }]
        ),
        # Include Gazebo launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_file)
        ),
        # Node to spawn the robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_pioneer',
            arguments=['-entity', 'pioneer3at', '-topic', 'robot_description', '-z', '0.051'],
            output='screen'
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
