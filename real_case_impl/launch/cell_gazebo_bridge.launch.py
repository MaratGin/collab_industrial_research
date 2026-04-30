from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_arguments = {
        "mqtt_host": LaunchConfiguration("mqtt_host"),
        "mqtt_port": LaunchConfiguration("mqtt_port"),
        "ur_robot_name": LaunchConfiguration("ur_robot_name"),
        "kuka_robot_name": LaunchConfiguration("kuka_robot_name"),
        "task_config_path": LaunchConfiguration("task_config_path"),
        "enable_visual_safety": LaunchConfiguration("enable_visual_safety"),
        "visual_safety_display_window": LaunchConfiguration("visual_safety_display_window"),
        "enable_operator_gui": LaunchConfiguration("enable_operator_gui"),
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("mqtt_host", default_value="localhost"),
            DeclareLaunchArgument("mqtt_port", default_value="1883"),
            DeclareLaunchArgument("ur_robot_name", default_value="ur5e"),
            DeclareLaunchArgument("kuka_robot_name", default_value="kuka"),
            DeclareLaunchArgument("task_config_path", default_value=""),
            DeclareLaunchArgument("enable_visual_safety", default_value="true"),
            DeclareLaunchArgument("visual_safety_display_window", default_value="false"),
            DeclareLaunchArgument("enable_operator_gui", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("real_case_impl"), "launch", "cell_sim.launch.py"]
                    )
                ),
                launch_arguments=launch_arguments.items(),
            )
        ]
    )
