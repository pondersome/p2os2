import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    cmd_motor_state_arg = LaunchConfiguration('cmd_motor_state', default='1')

    return LaunchDescription([
        DeclareLaunchArgument(
            'cmd_motor_state',
            default_value='1',
            description='Motor state to publish'
        ),
        ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '--once', '/cmd_motor_state', 'p2os_msgs/msg/MotorState', cmd_motor_state_arg],
            output='screen'
        )
    ])
