from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_type_arg = DeclareLaunchArgument("robot_type")
    debug_arg = DeclareLaunchArgument("debug", default_value="False")

    container = Node(
        package='rclcpp_components',
        executable='component_container_mt',
    )

    gary_common_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gary_bringup'),
                "launch",
                "gary_common",
                'gary_common.launch.py',
            ])
        ]), launch_arguments={"robot_type": LaunchConfiguration("robot_type")}.items()
    )

    gary_can_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gary_bringup'),
                "launch",
                "gary_can",
                'gary_can.launch.py',
            ])
        ]), launch_arguments={"robot_type": LaunchConfiguration("robot_type")}.items()
    )

    gary_serial_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gary_bringup'),
                "launch",
                "gary_serial",
                'gary_serial.launch.py',
            ])
        ]), launch_arguments={"robot_type": LaunchConfiguration("robot_type")}.items()
    )

    gary_chassis_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gary_bringup'),
                "launch",
                "gary_chassis",
                'gary_chassis.launch.py',
            ])
        ]), launch_arguments={"robot_type": LaunchConfiguration("robot_type")}.items()
    )

    ros2_control_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gary_bringup'),
                "launch",
                "ros2_control",
                'ros2_control.launch.py',
            ])
        ]), launch_arguments={"robot_type": LaunchConfiguration("robot_type")}.items()
    )

    debug_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gary_bringup'),
                "launch",
                "debug.launch.py",
            ])
        ]),
        launch_arguments={"robot_type": LaunchConfiguration("robot_type")}.items(),
        condition=IfCondition(PythonExpression([LaunchConfiguration("debug"), "== True"]))
    )

    description = [
        debug_arg,
        robot_type_arg,
        container,
        gary_common_description,
        gary_can_description,
        gary_serial_description,
        gary_chassis_description,
        ros2_control_description,
        debug_description,
    ]

    return LaunchDescription(description)
