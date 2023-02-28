from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_type_arg = DeclareLaunchArgument("robot_type")

    controller_base_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gary_bringup'),
                "launch",
                "ros2_control",
                'controller_base.launch.py',
            ])
        ]),
        launch_arguments={
            "urdf_name": PythonExpression(["'", LaunchConfiguration('robot_type'), ".urdf'"]),
            "robot_type": LaunchConfiguration("robot_type"),
        }.items(),
        condition=IfCondition(PythonExpression(
            ["__import__('os').path.exists(\"",
             PathJoinSubstitution([
                 FindPackageShare("gary_bringup"),
                 "config",
                 LaunchConfiguration("robot_type"),
                 "ros2_control",
                 "controller_base.yaml"
             ]),
             "\")"]
        ))
    )

    chassis_controllers_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gary_bringup'),
                "launch",
                "ros2_control",
                "chassis_controllers.launch.py"
            ])
        ]),
        condition=IfCondition(PythonExpression(
            ["__import__('os').path.exists(\"",
             PathJoinSubstitution([
                 FindPackageShare("gary_bringup"),
                 "config",
                 LaunchConfiguration("robot_type"),
                 "ros2_control",
                 "chassis_controllers.yaml"
             ]),
             "\")"]
        ))
    )

    gimbal_controllers_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gary_bringup'),
                "launch",
                "ros2_control",
                "gimbal_controllers.launch.py"
            ])
        ]),
        condition=IfCondition(PythonExpression(
            ["__import__('os').path.exists(\"",
             PathJoinSubstitution([
                 FindPackageShare("gary_bringup"),
                 "config",
                 LaunchConfiguration("robot_type"),
                 "ros2_control",
                 "gimbal_controllers.yaml"
             ]),
             "\")"]
        ))
    )

    liftup_controllers_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gary_bringup'),
                "launch",
                "ros2_control",
                "liftup_controllers.launch.py"
            ])
        ]),
        condition=IfCondition(PythonExpression(
            ["__import__('os').path.exists(\"",
             PathJoinSubstitution([
                 FindPackageShare("gary_bringup"),
                 "config",
                 LaunchConfiguration("robot_type"),
                 "ros2_control",
                 "liftup_controllers.yaml"
             ]),
             "\")"]
        ))
    )

    description = [
        robot_type_arg,
        controller_base_description,
        chassis_controllers_description,
        gimbal_controllers_description,
        liftup_controllers_description,
    ]

    return LaunchDescription(description)
