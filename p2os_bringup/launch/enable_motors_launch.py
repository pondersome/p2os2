import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression
)

def generate_launch_description():
    # LaunchConfigurations (the raw placeholders for runtime values)
    prefix_arg = LaunchConfiguration('prefix')
    cmd_motor_state_arg = LaunchConfiguration('cmd_motor_state')

    # Build the topic name via PythonExpression at runtime
    # If prefix is non-empty, produce:  "/prefix/cmd_motor_state"
    # Otherwise, produce:              "/cmd_motor_state"
    topic_name_expr = PythonExpression([
        "'/' + '",
        prefix_arg,
        "' + '/cmd_motor_state' if '",
        prefix_arg,
        "' != '' else '/cmd_motor_state'"
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'cmd_motor_state',
            default_value='1',
            description='Motor state to publish'
        ),
        DeclareLaunchArgument(
            'prefix',
            default_value='',
            description='Optional namespace for the topic'
        ),

        # Just to show the motor state we are using
        LogInfo(
            msg=['Publishing motor state: ', cmd_motor_state_arg]
        ),

        ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'pub', '--once',
                topic_name_expr,                 # <-- use the PythonExpression
                'p2os_msgs/msg/MotorState',
                # For the message data:
                PythonExpression(["'state: ' + str(", cmd_motor_state_arg, ")"])
            ],
            output='screen'
        )
    ])
