
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node, LoadComposableNodes, ComposableNodeContainer
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    name = LaunchConfiguration('name').perform(context)
    enable_battery_status = LaunchConfiguration(
        'enable_battery_status', default=True)
    run_container = LaunchConfiguration('run_container', default=True)
    return [
        ComposableNodeContainer(
            condition=IfCondition(run_container),
            name=name+'_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[],
            output='both',
        ),
        LoadComposableNodes(
            target_container=name+'_container',
            composable_node_descriptions=[
                ComposableNode(
                    name='battery_node',
                    package='rae_hw',
                    plugin='rae_hw::BatteryNode',
                ),
                ComposableNode(
                    name='lcd_node',
                    package='rae_hw',
                    plugin='rae_hw::LCDNode',
                    parameters=[{
                        'default_logo_path': LaunchConfiguration('default_logo_path')
                    }],
                ),
                ComposableNode(
                    name='led_node',
                    package='rae_hw',
                    plugin='rae_hw::LEDNode',
                )
            ]),
        Node(
            package='rae_hw',
            executable='mic_node'
        ),
        Node(
            package='rae_hw',
            executable='speakers_node'
        ),
        Node(
            package='rae_bringup',
            executable='battery_status.py',
            condition=IfCondition(enable_battery_status)
        ),
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('name', default_value='rae'),
        DeclareLaunchArgument('run_container', default_value='true'),
        DeclareLaunchArgument('enable_battery_status', default_value='true'),
        DeclareLaunchArgument('default_logo_path', default_value='/usr/share/rae-logo-white.jpg'),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
