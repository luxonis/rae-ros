import os
from ament_index_python.packages import get_package_share_path
import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition

def generate_launch_description():
    bringup_prefix = get_package_share_path('rae_bringup')
    bridge_prefix = get_package_share_path('rosbridge_server')
    enable_slam_toolbox = LaunchConfiguration('enable_slam_toolbox', default='true')
    enable_rosbridge = LaunchConfiguration('enable_rosbridge', default='false')
    return launch.LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_prefix, 'launch', 'robot.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_prefix, 'launch', 'slam.launch.py')),
            condition=IfCondition(enable_slam_toolbox)
            ),
    IncludeLaunchDescription(
            os.path.join(bridge_prefix, 'launch', 'rosbridge_websocket_launch.xml'),
            condition=IfCondition(enable_rosbridge)
            )
            ])
