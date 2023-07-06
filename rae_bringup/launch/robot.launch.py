import os
from ament_index_python.packages import get_package_share_path
import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    camera_prefix = get_package_share_path('rae_camera')
    hw_prefix = get_package_share_path('rae_hw')
    return launch.LaunchDescription([
        IncludeLaunchDescription(
<<<<<<< HEAD
        PythonLaunchDescriptionSource(
            os.path.join(camera_prefix, 'launch', 'camera.launch.py'))),
            IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hw_prefix, 'launch', 'control.launch.py'))),
    ])
=======
            PythonLaunchDescriptionSource(
                os.path.join(camera_prefix, 'launch', 'camera.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(hw_prefix, 'launch', 'control.launch.py'))),
    ])
>>>>>>> b5c1410de4d7aae6afc19dd6060d4f5132835a1d
