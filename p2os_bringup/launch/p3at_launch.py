import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare the launch arguments
    launch_arguments = [
        DeclareLaunchArgument('HokuyoLaser', default_value='0'),
        DeclareLaunchArgument('SICKLMSLaser', default_value='0'),
        DeclareLaunchArgument('P2OS_Driver', default_value='1'),
        DeclareLaunchArgument('KeyboardTeleop', default_value='0'),
        DeclareLaunchArgument('JoystickTeleop', default_value='0'),
        DeclareLaunchArgument('Transform', default_value='0'),
        DeclareLaunchArgument('enableMotor', default_value='1'),
        # P2OS driver parameters
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0', description='Serial port for ARCOS controller'),
        DeclareLaunchArgument('use_sonar', default_value='True', description='Enable sonar array'),
        DeclareLaunchArgument('baud_rate', default_value='0', description='Serial baud rate (9600/19200/38400/57600/115200). 0 = use robot model default'),
        DeclareLaunchArgument('max_xspeed', default_value='0.5', description='Max translational velocity (m/s). Firmware ceiling: 1.5 m/s'),
        DeclareLaunchArgument('max_yawspeed', default_value='1.7453', description='Max rotational velocity (rad/s). Firmware default: ~1.75 rad/s. Ceiling: 6.28 rad/s'),
        DeclareLaunchArgument('max_xaccel', default_value='0.0', description='Translational acceleration (m/s²). 0.0 = firmware default. Ceiling: 2.0 m/s²'),
        DeclareLaunchArgument('max_xdecel', default_value='0.0', description='Translational deceleration (m/s²). 0.0 = firmware default. Ceiling: 2.0 m/s²'),
        DeclareLaunchArgument('max_yawaccel', default_value='0.0', description='Rotational acceleration (rad/s²). 0.0 = firmware default. Ceiling: 5.24 rad/s²'),
        DeclareLaunchArgument('max_yawdecel', default_value='0.0', description='Rotational deceleration (rad/s²). 0.0 = firmware default. Ceiling: 5.24 rad/s²'),
    ]

    # Define the included launch descriptions with conditions
    included_launches = [
        GroupAction(
            actions=[
                Node(
                    package='p2os_driver',
                    executable='p2os_driver',
                    name='p2os_driver',
                    parameters=[
                        {'use_sonar': LaunchConfiguration('use_sonar')},
                        {'port': LaunchConfiguration('port')},
                        {'baud_rate': LaunchConfiguration('baud_rate')},
                        {'max_xspeed': LaunchConfiguration('max_xspeed')},
                        {'max_yawspeed': LaunchConfiguration('max_yawspeed')},
                        {'max_xaccel': LaunchConfiguration('max_xaccel')},
                        {'max_xdecel': LaunchConfiguration('max_xdecel')},
                        {'max_yawaccel': LaunchConfiguration('max_yawaccel')},
                        {'max_yawdecel': LaunchConfiguration('max_yawdecel')},
                    ]
                )
            ],
            condition=IfCondition(LaunchConfiguration('P2OS_Driver'))
        ),
        GroupAction(
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        FindPackageShare('p2os_bringup'), 'launch', 'teleop_keyboard_launch.py'
                    ])
                )
            ],
            condition=IfCondition(LaunchConfiguration('KeyboardTeleop'))
        ),
        GroupAction(
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        FindPackageShare('p2os_bringup'), 'launch', 'teleop_joy_launch.py'
                    ])
                )
            ],
            condition=IfCondition(LaunchConfiguration('JoystickTeleop'))
        ),
        GroupAction(
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        FindPackageShare('p2os_bringup'), 'launch', 'tf_base_link_to_laser_launch.py'
                    ])
                )
            ],
            condition=IfCondition(LaunchConfiguration('Transform'))
        ),
        GroupAction(
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        FindPackageShare('p2os_bringup'), 'launch', 'hokuyo_launch.py'
                    ])
                )
            ],
            condition=IfCondition(LaunchConfiguration('HokuyoLaser'))
        ),
        GroupAction(
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        FindPackageShare('p2os_bringup'), 'launch', 'sicklms_launch.py'
                    ])
                )
            ],
            condition=IfCondition(LaunchConfiguration('SICKLMSLaser'))
        ),
        GroupAction(
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([
                            FindPackageShare('p2os_bringup'), 'launch', 'enable_motors_launch.py'
                        ])
                    ])
                )
            ],
            condition=IfCondition(LaunchConfiguration('enableMotor'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('p2os_urdf'), 'launch', 'pioneer3at_urdf_launch.py'
                ])
            ])
        )
    ]

    return LaunchDescription(launch_arguments + included_launches)

if __name__ == '__main__':
    generate_launch_description()
