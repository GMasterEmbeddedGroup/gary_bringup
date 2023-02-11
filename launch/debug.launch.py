from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    robot_type_arg = DeclareLaunchArgument("robot_type")

    logger_config = LoadComposableNodes(
        target_container='/ComponentManager',
        composable_node_descriptions=[
            ComposableNode(
                package='logging_demo',
                plugin='logging_demo::LoggerConfig',
            ),
        ],
    )

    rosbridge_server_description = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rosbridge_server'),
                "launch",
                'rosbridge_websocket_launch.xml',
            ])
        ])
    )

    description = [
        robot_type_arg,
        logger_config,
        rosbridge_server_description,
    ]

    return LaunchDescription(description)
