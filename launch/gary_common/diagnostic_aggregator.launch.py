from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, PythonExpression, LaunchConfiguration
from launch.conditions import IfCondition

from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    robot_type_arg = DeclareLaunchArgument("robot_type")

    diagnostic_aggregator_params = PathJoinSubstitution(
        [
            FindPackageShare("gary_bringup"),
            "config",
            LaunchConfiguration("robot_type"),
            "gary_common",
            "diagnostic_aggregator.yaml",
        ]
    )

    diagnostic_aggregator = LoadComposableNodes(
        target_container='/ComponentManager',
        composable_node_descriptions=[
            ComposableNode(
                package='gary_common',
                plugin='gary_common::DiagnosticAggregator',
                parameters=[diagnostic_aggregator_params],
            ),
        ],
        condition=IfCondition(PythonExpression(
            ["__import__('os').path.exists(\"",
             PathJoinSubstitution([
                 FindPackageShare("gary_bringup"),
                 "config",
                 LaunchConfiguration("robot_type"),
                 "gary_common",
                 "diagnostic_aggregator.yaml"
             ]),
             "\")"]
        ))
    )

    description = [
        robot_type_arg,
        diagnostic_aggregator,
    ]

    return LaunchDescription(description)
