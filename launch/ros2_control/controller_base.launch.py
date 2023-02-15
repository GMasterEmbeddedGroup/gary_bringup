from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, PythonExpression

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    urdf_name_arg = DeclareLaunchArgument("urdf_name")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("gary_description"),
                    "urdf",
                    "ros2_control",
                    LaunchConfiguration("urdf_name"),
                ]
            ),
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    controller_manager_params = PathJoinSubstitution(
        [
            FindPackageShare("gary_bringup"),
            "config",
            LaunchConfiguration("robot_type"),
            "ros2_control",
            "controller_base.yaml",
        ]
    )

    config_found = IfCondition(PythonExpression(
                    ["__import__('os').path.exists(\"",
                     PathJoinSubstitution([
                         FindPackageShare("gary_bringup"),
                         "config",
                         LaunchConfiguration("robot_type"),
                         "ros2_control",
                         "controller_base.yaml"
                     ]),
                     "\")"]
                    ))

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_manager_params],
        condition=config_found,
    )

    offline_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["offline_broadcaster", "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "1",
                   "--unload-on-kill"],
        condition=config_found,
        respawn=True,
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "1",
                   "--unload-on-kill"],
        condition=config_found,
        respawn=True,
    )

    description = [
        urdf_name_arg,
        controller_manager,
        offline_broadcaster_spawner,
        joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(description)
