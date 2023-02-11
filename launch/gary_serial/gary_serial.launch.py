from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    robot_type_arg = DeclareLaunchArgument("robot_type")

    dr16_receiver_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gary_bringup'),
                "launch",
                "gary_serial",
                'dr16_receiver.launch.py',
            ])
        ]),
        launch_arguments={"robot_type": LaunchConfiguration("robot_type")}.items(),
    )

    description = [
        robot_type_arg,
        dr16_receiver_description,
    ]

    return LaunchDescription(description)
