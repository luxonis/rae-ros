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
<<<<<<< HEAD
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan_node',
            remappings=[('depth', '/rae/stereo_front/image_raw'),
                        ('depth_camera_info', '/rae/stereo_front/camera_info')],
            parameters=[{'output_frame': 'base_link',
                        'range_max': 20.0,
                        'scan_row': 250}]
        ),
=======
>>>>>>> 4426c67e0bb2cb3bc6888ca1ced2a8102e19fd3c
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(camera_prefix, 'launch', 'camera.launch.py'))),
            IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hw_prefix, 'launch', 'rae_control.launch.py'))),
    ])