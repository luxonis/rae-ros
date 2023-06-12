import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    rae_description_path = os.path.join(get_package_share_directory('rae_description'), 'urdf', 'rae.urdf.xacro')
    rae_description_config = xacro.process_file(rae_description_path)
    robot_description = {'robot_description': rae_description_config.toxml()}

    test_controller = os.path.join(get_package_share_directory('rae_hw'), 'config', 'controller.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('rae_description'), 'launch', 'rsp.launch.py')),
            launch_arguments={"sim": "false"}.items()
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[robot_description, test_controller],
            remappings=[('/diff_controller/cmd_vel_unstamped', 'cmd_vel')],
            output={
                'stdout': 'screen',
                'stderr': 'screen',
            },
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["diff_controller", "-c", "/controller_manager"],
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster",
                       "--controller-manager", "/controller_manager"],
        ),
        Node(
            package="rae_hw",
            executable="battery",
        ),
    ])