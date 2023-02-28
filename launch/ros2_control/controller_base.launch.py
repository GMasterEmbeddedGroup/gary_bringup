from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def controller_spawner(controller_name: str, controller_type: str, delay=0.0, condition=1):
    if condition == 1:
        return Node(
            package="controller_manager",
            executable="spawner.py",
            arguments=[controller_name,
                       "--controller-manager", "/controller_manager",
                       "--controller-type", controller_type,
                       ],
            on_exit=lambda event, context: TimerAction(
                period=delay,
                actions=[LogInfo(msg="{} spawn failed, retry in {} seconds".format(controller_name.strip(), delay)),
                         controller_spawner(controller_name, controller_type, delay + 1.0, event.returncode)],
            )
        )
    else:
        return LogInfo(msg="{} spawn successfully".format(controller_name.strip()))


def controller_manager():
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

    return Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_manager_params],
    )


def generate_launch_description():
    urdf_name_arg = DeclareLaunchArgument("urdf_name")

    description = [
        urdf_name_arg,
        controller_manager(),
        controller_spawner("offline_broadcaster", "gary_controller/OfflineBroadcaster"),
        controller_spawner("joint_state_broadcaster", "joint_state_broadcaster/JointStateBroadcaster"),
    ]

    return LaunchDescription(description)
