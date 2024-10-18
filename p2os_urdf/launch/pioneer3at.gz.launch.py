import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = 'p2os_urdf'

    # Define paths
    package_share = FindPackageShare(package=package_name)
    # Remove the final directory from the package_share path
    gz_resource_path = PythonExpression(["'", package_share, "'", ".rstrip('/').rsplit('/', 1)[0]"])
    defs_path = PathJoinSubstitution([package_share, 'defs'])
    models_path = '/home/karim/ros2_ws/install/p2os_urdf/share'
    meshes_path = PathJoinSubstitution([package_share, 'meshes'])
    worlds_path = PathJoinSubstitution([package_share, 'worlds'])
    
    # World file to load
    gz_sim_version = '8'  # Replace with your GZ Sim version 7=Garden, 8=Harmonic
    # Empty world from binary install
    world_file = PathJoinSubstitution([f'/usr/share/gz/gz-sim{gz_sim_version}/worlds', 'empty.sdf'])
    #world_file = PathJoinSubstitution([f'/usr/share/gz/gz-sim{gz_sim_version}/worlds', 'tunnel.sdf'])

    # Declare the launch argument for GZ_SIM_RESOURCE_PATH
    declare_gz_sim_resource_path_arg = DeclareLaunchArgument(
        'GZ_SIM_RESOURCE_PATH',
        default_value=''
    )

    # Set the GZ_SIM_RESOURCE_PATH environment variable
    set_gz_sim_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            LaunchConfiguration('GZ_SIM_RESOURCE_PATH'),
            os.pathsep, gz_resource_path
            #,defs_path, os.pathsep ,models_path, os.pathsep
            #, meshes_path, os.pathsep, worlds_path
        ]
    )

    # Generate robot description using xacro
    robot_description_command = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([package_share, 'defs', 'pioneer3at.xacro'])
    ])

    # Path to Gazebo empty world launch file
    gazebo_launch_file = PathJoinSubstitution([
        FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'
    ])



    #opaque function to print resource path for confirmation
    def log_environment_variable(context, *args, **kwargs):
        gz_sim_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
        print(f'GZ_SIM_RESOURCE_PATH: {gz_sim_resource_path}')
    

    return LaunchDescription([
        declare_gz_sim_resource_path_arg,
        set_gz_sim_resource_path,
        OpaqueFunction(function=log_environment_variable),
        # Node to publish robot state
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description_command,
                'publish_frequency': 30.0,
                'tf_prefix': ''
            }]
        ),
        # Node for p2os wheel joint publisher
        Node(
            package='p2os_urdf',
            executable='p2os_publisher_3at',
            name='publisher',
            output='screen'
        ),
        # Include Gazebo launch file with chosen world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_file),
            launch_arguments={'gz_args': world_file}.items()
        ),
        # Node to spawn the robot in Gazebo
        Node(
            package='ros_gz_sim',
            executable='create',
            name='spawn_pioneer',
            arguments=['-name', 'pioneer3at', '-topic', 'robot_description', '-z', '0.051'],
            output='screen'
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
