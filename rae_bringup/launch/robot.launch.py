import os
from ament_index_python.packages import get_package_share_path
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


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
                os.path.join(camera_prefix, 'launch', 'rae_camera.launch.py'))),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(hw_prefix, 'launch', 'control.launch.py')),
            launch_arguments={'run_container': 'false',
                              'enable_battery_status': 'true',
                              'enable_localization': 'true'}.items())
    ])
>>>>>>> b5c1410de4d7aae6afc19dd6060d4f5132835a1d
