import os
from ament_index_python.packages import get_package_share_path
import launch
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    bringup_prefix = get_package_share_path('rae_bringup')
    bridge_prefix = get_package_share_path('rosbridge_server')
    enable_slam_toolbox = LaunchConfiguration(
        'enable_slam_toolbox', default='true')
    enable_rosbridge = LaunchConfiguration('enable_rosbridge', default='false')
    enable_rtabmap = LaunchConfiguration('enable_rtabmap', default='false')
    enable_nav = LaunchConfiguration('enable_nav', default='false')
    nav_prefix = os.path.join(get_package_share_path('nav2_bringup'), 'launch')
    params = os.path.join(bringup_prefix, 'config', 'slam_param.yaml')

    return launch.LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_prefix, 'launch', 'robot.launch.py'))),
        IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(
                        nav_prefix, 'navigation_launch.py')),
                    launch_arguments={
                        'use_sim_time': 'false',
                        'params_file': params,
                        'use_composition': 'True',
                        'container_name': 'rae_container'}.items(),
                    condition=IfCondition(enable_nav)
                ),

        TimerAction(
            period=10.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                os.path.join(bringup_prefix, 'launch', 'slam.launch.py')),
            condition=IfCondition(enable_slam_toolbox)
        ),
                IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_prefix, 'launch', 'rtabmap.launch.py')),
            condition=IfCondition(enable_rtabmap)
        ),
                IncludeLaunchDescription(
            os.path.join(bridge_prefix, 'launch',
                         'rosbridge_websocket_launch.xml'),
            condition=IfCondition(enable_rosbridge)
        ),
                ]),
    ])
