from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    IfElseSubstitution,
)
from launch_ros.substitutions import FindPackageShare
import os



def generate_launch_description():
    ros_gz_sim_pkg_path = get_package_share_directory('ros_gz_sim')
    pkg_path = FindPackageShare('gz_world_package')  # Replace with your own package name
    world_pkg_path = FindPackageShare('gz_world_package')
    gz_launch_path = PathJoinSubstitution([ros_gz_sim_pkg_path, 'launch', 'gz_sim.launch.py'])
    world_name = LaunchConfiguration("world_name")
    bridge_file_path = LaunchConfiguration('bridge_file_path')
    world_path = PathJoinSubstitution([world_pkg_path, 'worlds', world_name])

    gzserver_cmd = IncludeLaunchDescription(
		os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'),
		launch_arguments={'gz_args': ['-r -s -v 1 --physics-engine gz-physics-bullet-featherstone-plugin ', world_path],
						  'on_exit_shutdown': 'true'}.items()
	)

    gzclient_cmd = IncludeLaunchDescription(
		os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'),
		launch_arguments={'gz_args': '-g -v4 '}.items()
	)

    return LaunchDescription([
        DeclareLaunchArgument(
		name='bridge_file_path',
		default_value=os.path.join(get_package_share_directory('gz_world_package'), 'config', 'gz_bridge.yaml'),
		description='Bridge configuration'
	),
        DeclareLaunchArgument(
            "world_name",
            description="GZ SIM world name.",
            default_value="robot_cell.sdf",
        ),
        gzserver_cmd,
        gzclient_cmd,
# LIRS_corridor.world
                # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(gz_launch_path),
        #     launch_arguments={
        #         'gz_args': [" -r -v 4 ",PathJoinSubstitution([pkg_path, 'worlds', world_name])],  # Replace with your own world file
        #         'on_exit_shutdown': 'True'
        #     }.items(),
        # ),
        

        # Bridging and remapping Gazebo topics to ROS 2 (replace with your own topics)
        # Node(
        # package="ros_gz_bridge",
        # executable="parameter_bridge",
        # arguments=[
        #     "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        # ],
        # output="screen",
        # ),
        # Node(
        #     package='ros_gz_bridge',
        #     executable='parameter_bridge',
        #     arguments=['/example_imu_topic@sensor_msgs/msg/Imu@gz.msgs.IMU',],
        #     remappings=[('/example_imu_topic',
        #                  '/remapped_imu_topic'),],
        #     output='screen'
        # ),
            # Spawn box in Gazebo
        Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_box',
        arguments=[
            '-file', PathJoinSubstitution([pkg_path, 'meshes', 'kuka_box', 'model.sdf']),
            '-name', 'simple_box',
            '-x', '-2.8',
            '-y', '0.5985',
            '-z', '0.3500',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '-1.5707'
        ],
        output='screen'
    ),
    Node(
		package='ros_gz_bridge',
		executable='parameter_bridge',
		output='screen',
		parameters=[{'config_file': os.path.join(get_package_share_directory('gz_world_package'), 'config', 'gz_bridge.yaml')}]
	),
        Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_ur_box',
        arguments=[
            '-file', PathJoinSubstitution([pkg_path, 'meshes', 'ur_box', 'model.sdf']),
            '-name', 'simple_ur_box',
            '-x', '-2.3',
            '-y', '-0.7771',
            '-z', '0.4000',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0'
        ],
        output='screen'
    ),
    #     Node(
    #     package='ros_gz_sim',
    #     executable='create',
    #     name='spawn_table',
    #     arguments=[
    #         '-file', PathJoinSubstitution([pkg_path, 'models', 'table', 'model.sdf']),
    #         '-name', 'table',
    #         '-x', '-3.2955',
    #         '-y', '-3.6271',
    #         '-z', '0.4000',
    #         '-R', '0.0',
    #         '-P', '0.0',
    #         '-Y', '0.0'
    #     ],
    #     output='screen'
    # ),

    ])