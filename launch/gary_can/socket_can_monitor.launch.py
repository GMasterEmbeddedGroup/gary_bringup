from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, PythonExpression, LaunchConfiguration

from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    robot_type_arg = DeclareLaunchArgument("robot_type")

    socket_can_monitor_params = PathJoinSubstitution(
        [
            FindPackageShare("gary_bringup"),
            "config",
            LaunchConfiguration("robot_type"),
            "gary_can",
            "socket_can_monitor.yaml",
        ]
    )

    socket_can_monitor = LoadComposableNodes(
        target_container='/ComponentManager',
        composable_node_descriptions=[
            ComposableNode(
                package='gary_can',
                plugin='driver::can::SocketCANMonitor',
                parameters=[socket_can_monitor_params],
            ),
        ],
        condition=IfCondition(PythonExpression(
            ["__import__('os').path.exists(\"",
             PathJoinSubstitution([
                 FindPackageShare("gary_bringup"),
                 "config",
                 LaunchConfiguration("robot_type"),
                 "gary_can",
                 "socket_can_monitor.yaml"
             ]),
             "\")"]
        ))
    )

    description = [
        robot_type_arg,
        socket_can_monitor,
    ]

    return LaunchDescription(description)
