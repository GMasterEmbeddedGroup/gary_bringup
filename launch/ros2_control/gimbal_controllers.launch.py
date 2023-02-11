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
        "gimbal_controllers.yaml",
    ])

    gimbal_imu_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["gimbal_imu_broadcaster",
                   "--controller-manager", "/controller_manager",
                   "--param-file", controllers_params,
                   "--controller-type", "imu_sensor_broadcaster/IMUSensorBroadcaster",
                   "--controller-manager-timeout", "30"],
    )

    description = [
        gimbal_imu_broadcaster_spawner,
    ]

    return LaunchDescription(description)
