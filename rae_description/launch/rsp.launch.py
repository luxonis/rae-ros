import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.conditions import IfCondition


def launch_setup(context, *args, **kwargs):
    name = LaunchConfiguration('name').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')
    run_container = LaunchConfiguration('run_container')
    pkg_path = os.path.join(get_package_share_directory('rae_description'))
    xacro_path = os.path.join(pkg_path, 'urdf', 'rae.urdf.xacro')

    return [
        ComposableNodeContainer(
            condition=IfCondition(run_container),
            name=name+"_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[],
            output="both",
        ),
        LoadComposableNodes(
        target_container=name+"_container",
        composable_node_descriptions=[
            ComposableNode(
                package='robot_state_publisher',
                plugin='robot_state_publisher::RobotStatePublisher',
                name=name+'_state_publisher',
                parameters=[{
                        'robot_description': Command(['xacro ', xacro_path, ' sim_mode:=', use_sim_time]),
                        'use_sim_time': use_sim_time
                }]
            )])
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
