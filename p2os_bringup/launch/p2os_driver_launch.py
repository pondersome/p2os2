from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # P2OS driver parameters
        # Firmware ceilings (values above these are silently clamped by ARCOS):
        #   transveltop=1.5 m/s, rotveltop=6.28 rad/s (360 deg/s)
        #   transacctop=2.0 m/s² (applies to both accel and decel)
        #   rotacctop=5.24 rad/s² / 300 deg/s² (applies to both accel and decel)
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0', description='Serial port for ARCOS controller'),
        DeclareLaunchArgument('use_sonar', default_value='False', description='Enable sonar array'),
        DeclareLaunchArgument('baud_rate', default_value='0', description='Serial baud rate (9600/19200/38400/57600/115200). 0 = use robot model default'),
        DeclareLaunchArgument('max_xspeed', default_value='0.5', description='Max translational velocity (m/s). Firmware ceiling: 1.5 m/s'),
        DeclareLaunchArgument('max_yawspeed', default_value='1.7453', description='Max rotational velocity (rad/s). Firmware default: ~1.75 rad/s. Ceiling: 6.28 rad/s'),
        DeclareLaunchArgument('max_xaccel', default_value='0.0', description='Translational acceleration (m/s²). 0.0 = firmware default. Ceiling: 2.0 m/s²'),
        DeclareLaunchArgument('max_xdecel', default_value='0.0', description='Translational deceleration (m/s²). 0.0 = firmware default. Ceiling: 2.0 m/s²'),
        DeclareLaunchArgument('max_yawaccel', default_value='0.0', description='Rotational acceleration (rad/s²). 0.0 = firmware default. Ceiling: 5.24 rad/s²'),
        DeclareLaunchArgument('max_yawdecel', default_value='0.0', description='Rotational deceleration (rad/s²). 0.0 = firmware default. Ceiling: 5.24 rad/s²'),

        Node(
            package='p2os_driver',
            executable='p2os_driver',
            name='p2os_driver',
            remappings=[
                ('pose', 'odom')
            ],
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
            ],
            arguments=['--ros-args', '--log-level', 'INFO']
        ),
    ])
