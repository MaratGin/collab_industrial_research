from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    IfElseSubstitution,
)
from launch_ros.substitutions import FindPackageShare

from launch.conditions import IfCondition, UnlessCondition
from launch_ros.parameter_descriptions import ParameterValue
import os
from launch.event_handlers import OnProcessExit
from nav2_common.launch import ReplaceString


def generate_launch_description():
    ld = LaunchDescription()
    kuka_namespace = LaunchConfiguration('kuka_namespace')
    ur_namespace = LaunchConfiguration('ur_namespace')
    conveyor_namespace = LaunchConfiguration('conveyor_namespace')

    robot_name = LaunchConfiguration('robot_name')
    use_sim_time = LaunchConfiguration('use_sim_time')

    camera = LaunchConfiguration("camera")


    kuka_declare_namespace_cmd = DeclareLaunchArgument(
		name='kuka_namespace',
		default_value='kuka',
		description='Top-level namespace'
	)
    ur_declare_namespace_cmd = DeclareLaunchArgument(
		name='ur_namespace',
		default_value='ur5e',
		description='Top-level namespace'
	)
    conveyor_declare_namespace_cmd = DeclareLaunchArgument(
		name='conveyor_namespace',
		default_value='conv',
		description='Top-level namespace'
	)

    declare_use_sim_time_cmd = DeclareLaunchArgument(
		name='use_sim_time',
		default_value='true',
		description='Use simulation (Gazebo) clock if true'
	)

    ur5e_spawn_launch = IncludeLaunchDescription(
        os.path.join(get_package_share_directory('ur5e_model_pkg'), 'launch', 'only_spawn_control.launch.py'),
        launch_arguments={
            'robot_namespace': ur_namespace,
            'use_sim_time': use_sim_time,
            'robot_name': 'ur5e',
            'prefix': '',
            'tf_prefix': 'ur5e_',
            'x': '-2.3',
            'y': '-0.7771',
            'z': '0.8000',
            'roll': '0',
            'pitch': '0',
            'yaw': '-1.570796',
            'use_sim_time': use_sim_time}.items()
    )

    kuka_spawn_launch = IncludeLaunchDescription(
        os.path.join(get_package_share_directory('kuka_kr3_model'), 'launch', 'only_spawn_control.launch.py'),
        launch_arguments={
            'robot_namespace': kuka_namespace,
            'use_sim_time': use_sim_time,
            'name': 'kuka',
            'tf_prefix': 'kuka_',
            'x': '-2.8',
            'y': '0.5985',
            'z': '0.72',
            'roll': '0',
            'pitch': '0',
            'yaw': '-1.0471',
            'use_sim_time': use_sim_time}.items()
    )

    conveyor_spawn_launch = IncludeLaunchDescription(
        os.path.join(get_package_share_directory('conveyor_pkg'), 'launch', 'spawn_conveyor.launch.py'),
        launch_arguments={
            'namespace': conveyor_namespace,
            'use_sim_time': use_sim_time,
            'robot_name': 'conveyor',
            'x': '-2.6',
            # 'x': '-22.6',
            'y': '-0.0342',
            'z': '0.05',
            'x_rot': '0',
            'y_rot': '0',
            'z_rot': '1.570796',
            'use_sim_time': use_sim_time}.items()
    )



    return LaunchDescription([
                              declare_use_sim_time_cmd,
                              kuka_declare_namespace_cmd,
                              ur_declare_namespace_cmd,
                              conveyor_declare_namespace_cmd,
                              ur5e_spawn_launch,
                              kuka_spawn_launch,
                              conveyor_spawn_launch,
                              ])
# ,
                            #   conveyor_spawn_launch