import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Publish the static transform from the center of the robot to the laser rangefinder (required for slam_gmapping)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            output='screen',
            arguments=['0', '0', '0.1397', '0', '0', '0', 'base_link', 'laser', '100']
        )
    ])
