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
    enable_operator_gui = LaunchConfiguration("enable_operator_gui")
    kuka_backend_cmd_topic = LaunchConfiguration("kuka_backend_cmd_topic")
    kuka_backend_status_topic = LaunchConfiguration("kuka_backend_status_topic")
    kuka_backend_event_topic = LaunchConfiguration("kuka_backend_event_topic")

    default_task_config = PathJoinSubstitution(
        [FindPackageShare("real_case_impl"), "config", "real_robot_task_sequences.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("mqtt_host", default_value="localhost"),
            DeclareLaunchArgument("mqtt_port", default_value="1883"),
            DeclareLaunchArgument("ur_robot_name", default_value="ur5e"),
            DeclareLaunchArgument("kuka_robot_name", default_value="kuka"),
            DeclareLaunchArgument("task_config_path", default_value=default_task_config),
            DeclareLaunchArgument("enable_operator_gui", default_value="true"),
            DeclareLaunchArgument("kuka_backend_cmd_topic", default_value="backend/kuka/cmd"),
            DeclareLaunchArgument("kuka_backend_status_topic", default_value="backend/kuka/status"),
            DeclareLaunchArgument("kuka_backend_event_topic", default_value="backend/kuka/event"),
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
                executable="ur_real_executor_node.py",
                name="ur_real_executor_node",
                output="screen",
                parameters=[
                    {"task_config_path": task_config_path},
                    {"robot_name": ur_robot_name},
                ],
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
                executable="kuka_real_bridge_node.py",
                name="kuka_real_bridge_node",
                output="screen",
                parameters=[
                    {"mqtt_host": mqtt_host},
                    {"mqtt_port": mqtt_port},
                    {"robot_name": kuka_robot_name},
                    {"backend_cmd_topic": kuka_backend_cmd_topic},
                    {"backend_status_topic": kuka_backend_status_topic},
                    {"backend_event_topic": kuka_backend_event_topic},
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
