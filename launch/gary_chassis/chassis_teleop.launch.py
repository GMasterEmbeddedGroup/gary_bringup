from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, PythonExpression, LaunchConfiguration

from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    robot_type_arg = DeclareLaunchArgument("robot_type")

    chassis_teleop_params = PathJoinSubstitution(
        [
            FindPackageShare("gary_bringup"),
            "config",
            LaunchConfiguration("robot_type"),
            "gary_chassis",
            "chassis_teleop.yaml",
        ]
    )

    chassis_teleop = LoadComposableNodes(
        target_container='/ComponentManager',
        composable_node_descriptions=[
            ComposableNode(
                package='gary_chassis',
                plugin='gary_chassis::ChassisTeleop',
                parameters=[chassis_teleop_params],
            ),
        ],
        condition=IfCondition(PythonExpression(
            ["__import__('os').path.exists(\"",
             PathJoinSubstitution([
                 FindPackageShare("gary_bringup"),
                 "config",
                 LaunchConfiguration("robot_type"),
                 "gary_chassis",
                 "chassis_teleop.yaml"
             ]),
             "\")"]
        ))
    )

    description = [
        robot_type_arg,
        chassis_teleop,
    ]

    return LaunchDescription(description)
