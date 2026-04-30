from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mqtt_host = LaunchConfiguration("mqtt_host")
    mqtt_port = LaunchConfiguration("mqtt_port")
    ur_robot_name = LaunchConfiguration("ur_robot_name")
    kuka_robot_name = LaunchConfiguration("kuka_robot_name")

    return LaunchDescription(
        [
            DeclareLaunchArgument("mqtt_host", default_value="localhost"),
            DeclareLaunchArgument("mqtt_port", default_value="1883"),
            DeclareLaunchArgument("ur_robot_name", default_value="ur5e"),
            DeclareLaunchArgument("kuka_robot_name", default_value="kuka"),
            Node(
                package="real_case_impl",
                executable="operator_gui_app.py",
                name="operator_gui_app",
                output="screen",
                parameters=[
                    {"mqtt_host": mqtt_host},
                    {"mqtt_port": mqtt_port},
                    {"ur_robot_name": ur_robot_name},
                    {"kuka_robot_name": kuka_robot_name},
                ],
            ),
        ]
    )
