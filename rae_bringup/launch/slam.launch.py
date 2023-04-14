import os
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration("params_file")
    return [

        IncludeLaunchDescription(os.path.join(get_package_share_path('nav2_bringup'), 'launch', 'slam_launch.py'),
        launch_arguments={
        'params_file': params_file}.items())
    ]
def generate_launch_description():
    bringup_prefix = get_package_share_path('rae_bringup')
    rviz_config = os.path.join(bringup_prefix, "config", "sim.rviz")
    declared_arguments = [
        DeclareLaunchArgument('sim`', default_value='False'),
        DeclareLaunchArgument('use_rviz', default_value='False'),
        DeclareLaunchArgument('rviz_config', default_value=rviz_config),
        DeclareLaunchArgument('params_file', default_value=os.path.join(bringup_prefix, 'config', 'sim.yaml')),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
