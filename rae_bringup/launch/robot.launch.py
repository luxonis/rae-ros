import os
from ament_index_python.packages import get_package_share_path
import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    bringup_prefix = get_package_share_path('rae_bringup')
    hw_prefix = get_package_share_path('rae_hw')
    DeclareLaunchArgument('use_sim_time', default_value='False')
    params = os.path.join(bringup_prefix, 'config', 'sim.yaml')
    return launch.LaunchDescription([
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
        IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_prefix, 'launch', 'rae_camera.launch.py'))),
            IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(hw_prefix, 'launch', 'rae_control.launch.py'))),
    ])