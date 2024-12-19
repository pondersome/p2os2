import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Command to generate robot description using xacro
    robot_description_command = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([FindPackageShare('p2os_urdf'), 'defs', 'pioneer3at.xacro'])
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
        # Node for p2os wheel joint publisher
        Node(
            package='p2os_urdf',
            executable='p2os_publisher_3at',
            name='wheel_publisher',
            output='screen'
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
