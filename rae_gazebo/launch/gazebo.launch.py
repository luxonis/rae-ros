import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    rae_description_pkg = get_package_share_directory('rae_description')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')
    enable_localization = LaunchConfiguration(
        'enable_localization', default='true')
    world_file = LaunchConfiguration('sdf_file').perform(context)
    print(f'world_file: {world_file}')
    return [
        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=[
                str(Path(rae_description_pkg).parent.resolve())
            ]
        ),

        # Ignition Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
            ]),
            launch_arguments={
                'gz_args': world_file,
                'use_sim_time': 'True'
            }.items()
        ),

        # RSP Node
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(rae_description_pkg, 'launch', 'rsp.launch.py')
            ]),
            launch_arguments={
                'use_sim_time': 'True'
            }.items()
        ),

        # ros_gz_bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{
                'use_sim_time': True,
                'config_file': os.path.join(get_package_share_directory(
                    'rae_gazebo'),'config', 'gz_bridge.yaml')
            }],
            output='screen'
        ),

        # Ignition Gazebo - Spawn Entity
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', 'robot_description',
            ],
            output='screen'
        ),

        # Activate diff controller
        Node(
            package='controller_manager',
            namespace=LaunchConfiguration('namespace'),
            executable='spawner',
            arguments=['diff_controller', '-c', '/controller_manager'],
        ),

        # Activate joint state broadcaster
        Node(
            package='controller_manager',
            executable='spawner',
            namespace=LaunchConfiguration('namespace'),
            arguments=['joint_state_broadcaster',
                    '--controller-manager', '/controller_manager'],
        ),

        # Activate EKF
        Node(
            condition=IfCondition(enable_localization),
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[os.path.join(get_package_share_directory(
                    'rae_hw'), 'config', 'ekf.yaml')],
        )
    ]



def generate_launch_description():
    rae_gazebo_pkg = get_package_share_directory('rae_gazebo')
    declared_arguments = [
        DeclareLaunchArgument('sdf_file', default_value=f"-r {os.path.join(rae_gazebo_pkg, 'worlds', 'world_demo.sdf')}"),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )