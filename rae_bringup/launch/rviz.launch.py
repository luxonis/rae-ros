import os
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

    return [
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            )
    ]


def generate_launch_description():
    bringup_prefix = get_package_share_path('rae_bringup')
    rviz_config = os.path.join(bringup_prefix, "config", "sim.rviz")
    declared_arguments = [
        DeclareLaunchArgument('rviz_config', default_value=rviz_config),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
