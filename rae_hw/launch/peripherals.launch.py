
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node, LoadComposableNodes, ComposableNodeContainer, LifecycleNode
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    name = LaunchConfiguration('name').perform(context)
    run_container = LaunchConfiguration('run_container', default=True)
    return [
        ComposableNodeContainer(
            condition=IfCondition(run_container),
            name=name+'_container',
            namespace=LaunchConfiguration('namespace'),
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
                    namespace=LaunchConfiguration('namespace'),
                    package='rae_hw',
                    plugin='rae_hw::BatteryNode',
                ),
                ComposableNode(
                    name='lcd_node',
                    namespace=LaunchConfiguration('namespace'),
                    package='rae_hw',
                    plugin='rae_hw::LCDNode',
                ),
                ComposableNode(
                    name='led_node',
                    namespace=LaunchConfiguration('namespace'),
                    package='rae_hw',
                    plugin='rae_hw::LEDNode',
                )

            ]),
        LifecycleNode(
            package='rae_hw',
            executable='speakers_node',
            name='speakers_node',
            namespace=LaunchConfiguration('namespace')
        ),
        LifecycleNode(
            package='rae_hw',
            executable='mic_node',
            name='mic_node',
            namespace=LaunchConfiguration('namespace')
        ),
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('name', default_value='rae'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('run_container', default_value='true'),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
