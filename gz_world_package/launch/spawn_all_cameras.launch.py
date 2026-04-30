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
    top_namespace = LaunchConfiguration('top_namespace')
    left_namespace = LaunchConfiguration('left_namespace')
    right_namespace = LaunchConfiguration('right_namespace')

    robot_name = LaunchConfiguration('robot_name')
    use_sim_time = LaunchConfiguration('use_sim_time')

    camera = LaunchConfiguration("camera")


    # UR specific arguments
    declare_camera_cmd = DeclareLaunchArgument(
            "camera",
            default_value="kinect_camera.urdf.xacro",
            description="URDF/XACRO description file (absolute path) with the robot.",
        )

    top_declare_namespace_cmd = DeclareLaunchArgument(
		name='top_namespace',
		default_value='top',
		description='Top-level namespace'
	)
    left_declare_namespace_cmd = DeclareLaunchArgument(
		name='left_namespace',
		default_value='left',
		description='Top-level namespace'
	)
    right_declare_namespace_cmd = DeclareLaunchArgument(
		name='right_namespace',
		default_value='right',
		description='Top-level namespace'
	)


    declare_use_sim_time_cmd = DeclareLaunchArgument(
		name='use_sim_time',
		default_value='true',
		description='Use simulation (Gazebo) clock if true'
	)

    top_desc_launch = IncludeLaunchDescription(
        os.path.join(get_package_share_directory('camera_models'), 'launch', 'camera_description.launch.py'),
        launch_arguments={
            'namespace': top_namespace,
            'use_sim_time': use_sim_time}.items()
    )
    top_spawn_launch = IncludeLaunchDescription(
        os.path.join(get_package_share_directory('camera_models'), 'launch', 'spawn_camera.launch.py'),
        launch_arguments={
            'camera':'kinect_camera.urdf.xacro',
            'namespace': top_namespace,
            'use_sim_time': use_sim_time,
            'robot_name': 'topkinect',
            'x': '-2.1955',
            'y': '-0.0204',
            'z': '2.0',
            'x_rot': '1.570796',
            'y_rot': '0',
            'z_rot': '1.570796'
            }.items()
    )

    left_spawn_launch = IncludeLaunchDescription(
        os.path.join(get_package_share_directory('camera_models'), 'launch', 'spawn_camera.launch.py'),
        launch_arguments={
            'camera':'kinect_camera.urdf.xacro',
            'namespace': left_namespace,
            'use_sim_time': use_sim_time,
            'robot_name': 'leftkinect',
            'x': '-1.87',
            'y': '2.22',
            'z': '1.06',
            'x_rot': '0',
            'y_rot': '0',
            'z_rot': '0'
            }.items()
    )

    right_spawn_launch = IncludeLaunchDescription(
        os.path.join(get_package_share_directory('camera_models'), 'launch', 'spawn_camera.launch.py'),
        launch_arguments={
            'camera':'kinect_camera.urdf.xacro',
            'namespace': right_namespace,
            'use_sim_time': use_sim_time,
            'robot_name': 'rightkinect',
            'x': '-1.73',
            'y': '-2.5478',
            'z': '1.06',
            'x_rot': '0',
            'y_rot': '0',
            'z_rot': '-3.141592'
            }.items()
    )




    return LaunchDescription([declare_camera_cmd,
                              declare_use_sim_time_cmd,
                              top_declare_namespace_cmd,  
                              left_declare_namespace_cmd,
                              right_declare_namespace_cmd,
                              top_desc_launch,
                              top_spawn_launch,
                              left_spawn_launch,
                              right_spawn_launch
                              ])
