from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    return LaunchDescription([
        # Teleop velocity limits
        DeclareLaunchArgument('max_vx', default_value='0.6', description='Max linear velocity (m/s)'),
        DeclareLaunchArgument('max_vx_turbo', default_value='0.6', description='Max linear velocity when turbo button held (m/s)'),
        DeclareLaunchArgument('max_vw', default_value='0.8', description='Max angular velocity (rad/s)'),
        DeclareLaunchArgument('max_vw_turbo', default_value='0.8', description='Max angular velocity when turbo button held (rad/s)'),
        DeclareLaunchArgument('teleop_rate', default_value='10', description='Teleop publish loop rate (Hz)'),

        Node(
            package='p2os_teleop',
            executable='p2os_teleop',
            name='p2os_teleop',
            remappings=[
                ('/des_vel', '/base_controller/command')
            ],
            parameters=[
                {'axis_vx': 1},
                {'axis_vw': 2},
                {'axis_vy': 0},
                #{'deadman_button': 5}, #logitech F310 xbox mode - bumpers
                #{'turbo_button': 4}, #logitech F310 xbox mode - bumpers
                {'deadman_button': 6}, #gamesir nova lite pc mode - left bumper
                {'turbo_button': 7}, #gamesir nova lite pc mode - right bumper
                {'max_vx': LaunchConfiguration('max_vx')},
                {'max_vx_turbo': LaunchConfiguration('max_vx_turbo')},
                {'max_vw': LaunchConfiguration('max_vw')},
                {'max_vw_turbo': LaunchConfiguration('max_vw_turbo')},
                {'teleop_rate': LaunchConfiguration('teleop_rate')},
            ]
        ),

        # Node for joystick control with device parameter from YAML
        Node(
            package='joy',
            executable='joy_node',
            name='joy_controller',
            #parameters=[joystick_config]
            parameters = [{'dev': '/dev/input/js0'}] #can override from command line, F310 is usually js1
        ),
    ])
