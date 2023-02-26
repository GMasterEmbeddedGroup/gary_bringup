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
                   "--controller-manager-timeout", "1",
                   "--unload-on-kill"],
        respawn=True,
    )

    gimbal_pitch_pid_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["gimbal_pitch_pid",
                   "--controller-manager", "/controller_manager",
                   "--param-file", controllers_params,
                   "--controller-type", "gary_controller/DualLoopPIDControllerWithFilter",
                   "--controller-manager-timeout", "1",
                   "--unload-on-kill"],
        respawn=True,
    )

    gimbal_yaw_pid_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["gimbal_yaw_pid",
                   "--controller-manager", "/controller_manager",
                   "--param-file", controllers_params,
                   "--controller-type", "gary_controller/DualLoopPIDControllerWithFilter",
                   "--controller-manager-timeout", "1",
                   "--unload-on-kill"],
        respawn=True,
    )

    description = [
        gimbal_imu_broadcaster_spawner,
        gimbal_pitch_pid_spawner,
        gimbal_yaw_pid_spawner,
    ]

    return LaunchDescription(description)
