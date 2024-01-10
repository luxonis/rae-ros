import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
import xacro


def launch_setup(context, *args, **kwargs):
    rae_description_path = os.path.join(get_package_share_directory(
        'rae_description'), 'urdf', 'rae.urdf.xacro')
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('rae_description'), 'launch', 'rsp.launch.py')),
            launch_arguments={'sim': 'false',
                              'namespace': LaunchConfiguration('namespace'), 
                              }.items()
        ),
        Node(
            package='rae_hw',
            executable='mock_wheels.py',
            name='mock_wheels',
            namespace=LaunchConfiguration('namespace')
        ),
        Node(
            package='rae_hw',
            executable='mock_leds.py',
            name='mock_leds',
            namespace=LaunchConfiguration('namespace')
        ),
        Node(
            package='rae_hw',
            executable='mock_battery.py',
            name='mock_battery',
            namespace=LaunchConfiguration('namespace')
        ),
        Node(
            package='rae_hw',
            executable='mock_lcd.py',
            name='mock_lcd',
            namespace=LaunchConfiguration('namespace')
        ),
        Node(
            package='rae_hw',
            executable='mock_speakers.py',
            name='mock_speakers',
            namespace=LaunchConfiguration('namespace')
        ),
        Node(
            package='rae_hw',
            executable='mock_mic.py',
            name='mock_mic',
            namespace=LaunchConfiguration('namespace')
        )
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('name', default_value='rae'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('run_container', default_value='true'),
        DeclareLaunchArgument('enable_battery_status', default_value='true'),
        DeclareLaunchArgument('enable_localization', default_value='true'),
        DeclareLaunchArgument('controller_params_file', default_value=os.path.join(get_package_share_directory(
        'rae_hw'), 'config', 'controller.yaml')),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
