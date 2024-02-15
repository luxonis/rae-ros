import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node, LifecycleNode
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
            executable='lifecycle_manager.py',
            name='lifecycle_manager',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{'mock': True}]
        ),
        LifecycleNode(
            package='rae_hw',
            executable='mock_wheels.py',
            name='diff_controller',
            namespace=LaunchConfiguration('namespace')
        ),
        LifecycleNode(
            package='rae_hw',
            executable='mock_leds.py',
            name='led_node',
            namespace=LaunchConfiguration('namespace')
        ),
        LifecycleNode(
            package='rae_hw',
            executable='mock_battery.py',
            name='battery_node',
            namespace=LaunchConfiguration('namespace')
        ),
        LifecycleNode(
            package='rae_hw',
            executable='mock_lcd.py',
            name='lcd_node',
            namespace=LaunchConfiguration('namespace')
        ),
        LifecycleNode(
            package='rae_hw',
            executable='mock_speakers.py',
            name='speakers_node',
            namespace=LaunchConfiguration('namespace')
        ),
        LifecycleNode(
            package='rae_hw',
            executable='mock_mic.py',
            name='mic_node',
            namespace=LaunchConfiguration('namespace')
        ),
        LifecycleNode(
            package='rae_hw',
            executable='sys_info_node.py',
            name='sys_info',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{'mock': True}]
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
