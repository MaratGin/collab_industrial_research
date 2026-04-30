from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mqtt_host = LaunchConfiguration("mqtt_host")
    mqtt_port = LaunchConfiguration("mqtt_port")
    ur_robot_name = LaunchConfiguration("ur_robot_name")
    kuka_robot_name = LaunchConfiguration("kuka_robot_name")
    task_config_path = LaunchConfiguration("task_config_path")
    enable_visual_safety = LaunchConfiguration("enable_visual_safety")
    visual_safety_display_window = LaunchConfiguration("visual_safety_display_window")
    enable_operator_gui = LaunchConfiguration("enable_operator_gui")

    default_task_config = PathJoinSubstitution(
        [FindPackageShare("real_case_impl"), "config", "sim_robot_task_sequences.yaml"]
    )

    common_executor_params = [{"task_config_path": task_config_path}]

    return LaunchDescription(
        [
            DeclareLaunchArgument("mqtt_host", default_value="localhost"),
            DeclareLaunchArgument("mqtt_port", default_value="1883"),
            DeclareLaunchArgument("ur_robot_name", default_value="ur5e"),
            DeclareLaunchArgument("kuka_robot_name", default_value="kuka"),
            DeclareLaunchArgument("task_config_path", default_value=default_task_config),
            DeclareLaunchArgument("enable_visual_safety", default_value="true"),
            DeclareLaunchArgument("visual_safety_display_window", default_value="false"),
            DeclareLaunchArgument("enable_operator_gui", default_value="true"),
            Node(
                package="real_case_impl",
                executable="supervisor_node.py",
                name="cell_supervisor_node",
                output="screen",
                parameters=[
                    {"mqtt_host": mqtt_host},
                    {"mqtt_port": mqtt_port},
                    {"ur_robot_name": ur_robot_name},
                    {"kuka_robot_name": kuka_robot_name},
                ],
            ),
            Node(
                package="real_case_impl",
                executable="ur_adapter_node.py",
                name="ur_adapter_node",
                output="screen",
                parameters=[
                    {"mqtt_host": mqtt_host},
                    {"mqtt_port": mqtt_port},
                    {"robot_name": ur_robot_name},
                ],
            ),
            Node(
                package="real_case_impl",
                executable="ur_sim_executor_node.py",
                name="ur_sim_executor_node",
                output="screen",
                parameters=common_executor_params + [{"robot_name": ur_robot_name}],
            ),
            Node(
                package="real_case_impl",
                executable="kuka_adapter_node.py",
                name="kuka_adapter_node",
                output="screen",
                parameters=[
                    {"mqtt_host": mqtt_host},
                    {"mqtt_port": mqtt_port},
                    {"robot_name": kuka_robot_name},
                ],
            ),
            Node(
                package="real_case_impl",
                executable="kuka_sim_executor_node.py",
                name="kuka_sim_executor_node",
                output="screen",
                parameters=common_executor_params + [{"robot_name": kuka_robot_name}],
            ),
            Node(
                package="real_case_impl",
                executable="combined_safety_classifier.py",
                name="visual_safety_system",
                output="screen",
                condition=IfCondition(enable_visual_safety),
                parameters=[
                    {"display_window": visual_safety_display_window},
                ],
            ),
            Node(
                package="real_case_impl",
                executable="operator_gui_app.py",
                name="operator_gui_app",
                output="screen",
                condition=IfCondition(enable_operator_gui),
                parameters=[
                    {"mqtt_host": mqtt_host},
                    {"mqtt_port": mqtt_port},
                    {"ur_robot_name": ur_robot_name},
                    {"kuka_robot_name": kuka_robot_name},
                ],
            ),
        ]
    )

