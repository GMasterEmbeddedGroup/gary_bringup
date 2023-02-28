from launch import LaunchDescription
from launch.actions import LogInfo, TimerAction

from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def controllers_params():
    return PathJoinSubstitution([
        FindPackageShare("gary_bringup"),
        "config",
        LaunchConfiguration("robot_type"),
        "ros2_control",
        "gimbal_controllers.yaml",
    ])


def controller_spawner(controller_name: str, controller_type: str, delay=0.0, condition=1):
    if condition == 1:
        return Node(
            package="controller_manager",
            executable="spawner.py",
            arguments=[controller_name,
                       "--controller-manager", "/controller_manager",
                       "--param-file", controllers_params(),
                       "--controller-type", controller_type,
                       ],
            on_exit=lambda event, context: TimerAction(
                period=delay,
                actions=[LogInfo(msg="{} spawn failed, retry in {} seconds".format(controller_name.strip(), delay)),
                         controller_spawner(controller_name, controller_type, delay + 1.0, event.returncode)],
            )
        )
    else:
        return LogInfo(msg="{} spawn successfully".format(controller_name.strip()))


def generate_launch_description():

    description = [
        controller_spawner("gimbal_imu_broadcaster", "imu_sensor_broadcaster/IMUSensorBroadcaster"),
        controller_spawner("gimbal_pitch_pid", "gary_controller/DualLoopPIDControllerWithFilter"),
        controller_spawner("gimbal_yaw_pid", "gary_controller/DualLoopPIDControllerWithFilter"),
    ]

    return LaunchDescription(description)
