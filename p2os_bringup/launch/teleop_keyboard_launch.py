import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import SetParameter
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    pr2_teleop_launch = os.path.join(
        get_package_share_directory('pr2_teleop'),
        'launch',
        'teleop_keyboard.launch.py'
    )
    
    p2os_launch = os.path.join(
        get_package_share_directory('p2os_launch'),
        'launch',
        'teleop_joy.launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pr2_teleop_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(p2os_launch)
        ),
        SetParameter(name='/spawn_teleop_keyboard/walk_vel', value=0.25),
        SetParameter(name='/spawn_teleop_keyboard/run_vel', value=0.5),
        SetParameter(name='/spawn_teleop_keyboard/yaw_rate', value=0.5),
        SetParameter(name='/spawn_teleop_keyboard/yaw_run_rate', value=0.75)
    ])

#todo this will have to change when we upgrade to anything more current than foxy
def get_package_share_directory(package_name):
    return os.path.join('/opt/ros/foxy/share', package_name)
