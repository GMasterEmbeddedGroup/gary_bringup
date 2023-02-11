from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, PythonExpression, LaunchConfiguration

from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    robot_type_arg = DeclareLaunchArgument("robot_type")

    dr16_receiver_params = PathJoinSubstitution(
        [
            FindPackageShare("gary_bringup"),
            "config",
            LaunchConfiguration("robot_type"),
            "gary_serial",
            "dr16_receiver.yaml",
        ]
    )

    dr16_receiver = LoadComposableNodes(
        target_container='/ComponentManager',
        composable_node_descriptions=[
            ComposableNode(
                package='gary_serial',
                plugin='gary_serial::DR16Receiver',
                parameters=[dr16_receiver_params],
            ),
        ],
        condition=IfCondition(PythonExpression(
            ["__import__('os').path.exists(\"",
             PathJoinSubstitution([
                 FindPackageShare("gary_bringup"),
                 "config",
                 LaunchConfiguration("robot_type"),
                 "gary_serial",
                 "dr16_receiver.yaml"
             ]),
             "\")"]
        ))
    )

    description = [
        robot_type_arg,
        dr16_receiver,
    ]

    return LaunchDescription(description)
