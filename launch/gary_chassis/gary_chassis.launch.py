from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    robot_type_arg = DeclareLaunchArgument("robot_type")

    mecanum_chassis_solver_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gary_bringup'),
                "launch",
                "gary_chassis",
                'mecanum_chassis_solver.launch.py',
            ])
        ]),
        launch_arguments={"robot_type": LaunchConfiguration("robot_type")}.items(),
    )

    omnidirectional_chassis_solver_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gary_bringup'),
                "launch",
                "gary_chassis",
                'omnidirectional_chassis_solver.launch.py',
            ])
        ]),
        launch_arguments={"robot_type": LaunchConfiguration("robot_type")}.items(),
    )

    chassis_teleop_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gary_bringup'),
                "launch",
                "gary_chassis",
                'chassis_teleop.launch.py',
            ])
        ]),
        launch_arguments={"robot_type": LaunchConfiguration("robot_type")}.items(),
    )

    description = [
        robot_type_arg,
        mecanum_chassis_solver_description,
        omnidirectional_chassis_solver_description,
        chassis_teleop_description,
    ]

    return LaunchDescription(description)
