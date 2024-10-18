import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():
    cmd_motor_state_arg = LaunchConfiguration('cmd_motor_state')

    return LaunchDescription([
        DeclareLaunchArgument(
            'cmd_motor_state',
            default_value='1',
            description='Motor state to publish'
        ),
        LogInfo(
            msg=['Publishing motor state: ', cmd_motor_state_arg]
        ),
        ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'pub', '--once', '/cmd_motor_state', 'p2os_msgs/msg/MotorState',
                PythonExpression(["'state: ' + str(", cmd_motor_state_arg, ")"])
            ],
            output='screen'
        )
    ])
