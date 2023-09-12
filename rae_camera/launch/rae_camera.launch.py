import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, RegisterEventHandler, TimerAction, LogInfo
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    log_level = 'info'
    if (context.environment.get('DEPTHAI_DEBUG') == '1'):
        log_level = 'debug'
    params_file = LaunchConfiguration("params_file")
    name = LaunchConfiguration('name').perform(context)

    reset_pwm = ExecuteProcess(
        cmd=[['busybox devmem 0x20320180 32 0x00000000']],
        shell=True
    )

    perception = ComposableNodeContainer(
        name=name+"_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="depthai_ros_driver",
                plugin="depthai_ros_driver::Camera",
                name=name,
                parameters=[params_file],
            ),
        ],
        arguments=['--ros-args', '--log-level', log_level],
        output="both",
    )

    return [
        perception,
        RegisterEventHandler(
            OnProcessStart(
                target_action=perception,
                on_start=[
                    TimerAction(
                        period=15.0,
                        actions=[reset_pwm, LogInfo(msg='Resetting PWM.'),],
                    )
                ]
            )
        ),

    ]


def generate_launch_description():
    camera_prefix = get_package_share_directory("rae_camera")
    declared_arguments = [
        DeclareLaunchArgument("name", default_value="rae"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(
            camera_prefix, 'config', 'camera.yaml')),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
