from launch import LaunchDescription

from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    controllers_params = PathJoinSubstitution([
        FindPackageShare("gary_bringup"),
        "config",
        LaunchConfiguration("robot_type"),
        "ros2_control",
        "chassis_controllers.yaml",
    ])

    chassis_lf_pid_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["chassis_lf_pid",
                   "--controller-manager", "/controller_manager",
                   "--param-file", controllers_params,
                   "--controller-type", "gary_controller/PIDController",
                   "--controller-manager-timeout", "30"],
    )

    chassis_lb_pid_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["chassis_lb_pid",
                   "--controller-manager", "/controller_manager",
                   "--param-file", controllers_params,
                   "--controller-type", "gary_controller/PIDController",
                   "--controller-manager-timeout", "30"],
    )

    chassis_rf_pid_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["chassis_rf_pid",
                   "--controller-manager", "/controller_manager",
                   "--param-file", controllers_params,
                   "--controller-type", "gary_controller/PIDController",
                   "--controller-manager-timeout", "30"],
    )

    chassis_rb_pid_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["chassis_rb_pid",
                   "--controller-manager", "/controller_manager",
                   "--param-file", controllers_params,
                   "--controller-type", "gary_controller/PIDController",
                   "--controller-manager-timeout", "30"],
    )

    description = [
        chassis_lf_pid_spawner,
        chassis_lb_pid_spawner,
        chassis_rf_pid_spawner,
        chassis_rb_pid_spawner,
    ]

    return LaunchDescription(description)
