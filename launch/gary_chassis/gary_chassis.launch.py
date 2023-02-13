from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    robot_type_arg = DeclareLaunchArgument("robot_type")

    chassis_solver_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gary_bringup'),
                "launch",
                "gary_chassis",
                'chassis_solver.launch.py',
            ])
        ]),
        launch_arguments={"robot_type": LaunchConfiguration("robot_type")}.items(),
    )

    description = [
        robot_type_arg,
        chassis_solver_description,
    ]

    return LaunchDescription(description)
