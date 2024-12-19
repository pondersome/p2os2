import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution, TextSubstitution

def generate_launch_description():
    prefix_arg = LaunchConfiguration('prefix')
    cmd_motor_state_arg = LaunchConfiguration('cmd_motor_state')

    return LaunchDescription([
        DeclareLaunchArgument(
            'cmd_motor_state',
            default_value='1',
            description='Motor state to publish'
        ),
        DeclareLaunchArgument(
            'prefix', 
            default_value='', 
            description='Namespace for topic'
        ),
        LogInfo(
            msg=['Publishing motor state: ', cmd_motor_state_arg]
        ),
        ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'pub', '--once', 
                # Concatenate prefix_arg with "/cmd_motor_state" properly
                PathJoinSubstitution([
                    TextSubstitution(text='/'),  # Add a leading slash
                    prefix_arg,  # Namespace
                    TextSubstitution(text='cmd_motor_state')  # Topic name
                ]),
                'p2os_msgs/msg/MotorState',
                PythonExpression(["'state: ' + str(", cmd_motor_state_arg, ")"])
            ],
            output='screen'
        )
    ])
