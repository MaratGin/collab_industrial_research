from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def build_kuka_moveit_config():
    return (
        MoveItConfigsBuilder(
            robot_name="kuka_kr3",
            package_name="moveit_kuka_kr3_config",
        )
        .robot_description(
            file_path="config/kuka.urdf.xacro",
            mappings={
                "tf_prefix": "kuka_",
                "prefix": "kuka_",
                "namespace": "kuka",
            },
        )
        .robot_description_semantic(file_path="config/kuka.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )


def build_ur_moveit_config():
    return (
        MoveItConfigsBuilder(
            robot_name="ur5e",
            package_name="ur5e_2fg7_moveit_config",
        )
        .robot_description(
            file_path="config/ur5e.urdf.xacro",
            mappings={
                "tf_prefix": "ur5e_",
                "prefix": "ur5e_",
                "namespace": "ur5e",
            },
        )
        .robot_description_semantic(file_path="config/ur5e.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")

    kuka_config = build_kuka_moveit_config()
    ur_config = build_ur_moveit_config()

    kuka_move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        # namespace="kuka",
        output="screen",
        parameters=[
            kuka_config.to_dict(),
            ur_config.to_dict(),
            {"use_sim_time": use_sim_time},
        ],
        # remappings=[
        #     ("/joint_states", "/kuka/joint_states"),
        # ],
    )

    ur_move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        namespace="ur5e",
        output="screen",
        parameters=[
            ur_config.to_dict(),
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("/joint_states", "/ur5e/joint_states"),
        ],
    )

    kuka_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="kuka_rviz2",
        namespace="kuka",
        output="screen",
        condition=IfCondition(launch_rviz),
        arguments=[
            "-d",
            str(kuka_config.package_path / "config" / "moveit.rviz"),
        ],
        # arguments=[],
        parameters=[
            kuka_config.robot_description,
            kuka_config.robot_description_semantic,
            kuka_config.robot_description_kinematics,
            kuka_config.planning_pipelines,
            kuka_config.joint_limits,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("/joint_states", "/kuka/joint_states"),
        ],
    )

    # kuka_rviz = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="kuka_rviz2",
    #     output="screen",
    #     arguments=[],
    #     parameters=[
    #         kuka_config.to_dict(),
    #         {"use_sim_time": use_sim_time},
    #     ],
    #     remappings=[
    #         ("/joint_states", "/kuka/joint_states"),
    #     ],
    # )
    # ur_rviz = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="ur5e_rviz2_moveit",
    #     namespace="ur5e",
    #     output="screen",
    #     condition=IfCondition(launch_rviz),
    #     arguments=[
    #         "-d",
    #         str(ur_config.package_path / "config" / "moveit.rviz"),
    #     ],
    #     parameters=[
    #         ur_config.robot_description,
    #         ur_config.robot_description_semantic,
    #         ur_config.robot_description_kinematics,
    #         ur_config.planning_pipelines,
    #         ur_config.joint_limits,
    #         {"use_sim_time": use_sim_time},
    #     ],
    #     remappings=[
    #         ("/joint_states", "/ur5e/joint_states"),
    #     ],
    # )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
        ),

        kuka_move_group,
        # ur_move_group,

        kuka_rviz,
        # ur_rviz,
    ])