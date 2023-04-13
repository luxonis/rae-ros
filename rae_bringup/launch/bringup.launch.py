import os
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    LaunchConfiguration('sim')
    bringup_prefix = get_package_share_path('rae_bringup')
    param_file_name = 'sim.yaml'
    if not LaunchConfiguration('sim').perform(context):
        param_file_name = 'robot.yaml'

    params = os.path.join(bringup_prefix, 'config', param_file_name)

    return [

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_path('rae_bringup'), 'launch', 'rviz.launch.py')),
            condition=IfCondition(LaunchConfiguration(
                "use_rviz").perform(context)),
                launch_arguments={'rviz_config': LaunchConfiguration('rviz_config').perform(context)}.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_path('rae_gazebo'), 'launch', 'rae_simulation.launch.py')),
            condition=IfCondition(LaunchConfiguration("sim").perform(context))
        ),

        IncludeLaunchDescription(os.path.join(bringup_prefix, 'launch', 'slam.launch.py'),
                                 launch_arguments={
            'params_file': params}.items()),

        # IncludeLaunchDescription(os.path.join(bringup_prefix, 'launch', 'nav.launch.py'),
        #                          launch_arguments={
        #     'params_file': params}.items())
    ]


def generate_launch_description():
    bringup_prefix = get_package_share_path('rae_bringup')
    rviz_config = os.path.join(bringup_prefix, "config", "sim.rviz")
    declared_arguments = [
        DeclareLaunchArgument('sim', default_value='True'),
        DeclareLaunchArgument('use_rviz', default_value='False'),
        DeclareLaunchArgument('rviz_config', default_value=rviz_config),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
