import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from launch.conditions import IfCondition


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time')
    pkg_path = os.path.join(get_package_share_directory('rae_description'))
    xacro_path = os.path.join(pkg_path, 'urdf', 'rae.urdf.xacro')

    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': Command(['xacro ', xacro_path, ' sim_mode:=', use_sim_time]),
                'use_sim_time': use_sim_time
            }]
        )
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'name',
            default_value='rae',
            description='Robot name'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time'
        ),
        DeclareLaunchArgument(
            'run_container',
            default_value='false',
            description='Run container'
        ),
    ]
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
