import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
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
        DeclareLaunchArgument('Transform', default_value='1'),
        DeclareLaunchArgument('enableMotor', default_value='1'),
    ]

    # Define the included launch descriptions with conditions
    included_launches = [
        GroupAction(
            actions=[
                Node(
                    package='p2os_driver',
                    executable='p2os_driver',
                    name='p2os_driver',
                    parameters=[{'use_sonar': False}]
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
                        FindPackageShare('p2os_bringup'), 'launch', 'enable_motors_launch.py'
                    ])
                )
            ],
            condition=IfCondition(LaunchConfiguration('enableMotor'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('p2os_urdf'), 'launch', 'pioneer3at_urdf_launch.py'
            ])
        )
    ]

    return LaunchDescription(launch_arguments + included_launches)

if __name__ == '__main__':
    generate_launch_description()
