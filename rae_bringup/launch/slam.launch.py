import os
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration("params_file")
    rae_prefix = get_package_share_path('rae_bringup')
    return [
        
        Node(
        parameters=[
          params_file,
          {'use_sim_time': False}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')
    ]
def generate_launch_description():
    bringup_prefix = get_package_share_path('rae_bringup')
    rviz_config = os.path.join(bringup_prefix, "config", "sim.rviz")
    declared_arguments = [
        DeclareLaunchArgument('sim`', default_value='False'),
        DeclareLaunchArgument('use_rviz', default_value='False'),
        DeclareLaunchArgument('rviz_config', default_value=rviz_config),
        DeclareLaunchArgument('params_file', default_value=os.path.join(bringup_prefix, 'config', 'slam_param.yaml')),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )