from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    robot_type_arg = DeclareLaunchArgument("robot_type")

    diagnostic_aggregator_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gary_bringup'),
                "launch",
                "gary_common",
                'diagnostic_aggregator.launch.py',
            ])
        ]),
        launch_arguments={"robot_type": LaunchConfiguration("robot_type")}.items(),
    )

    lifecycle_manager_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gary_bringup'),
                "launch",
                "gary_common",
                'lifecycle_manager.launch.py',
            ])
        ]),
        launch_arguments={"robot_type": LaunchConfiguration("robot_type")}.items(),
    )

    description = [
        robot_type_arg,
        diagnostic_aggregator_description,
        lifecycle_manager_description
    ]

    return LaunchDescription(description)
