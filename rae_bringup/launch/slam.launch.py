import os
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration("params_file")
    return [
        # Node(
        #     package='depthimage_to_laserscan',
        #     executable='depthimage_to_laserscan_node',
        #     name='depthimage_to_laserscan_node',
        #     remappings=[('depth', '/rae/stereo_front/image_raw'),
        #                 ('depth_camera_info', '/rae/stereo_front/camera_info')],
        #     parameters=[{'output_frame': 'base_link',
        #                 'range_max': 20.0,
        #                 'scan_row': 250}]
        # ),
        # Node(
        #     package='laserscan_kinect',
        #     executable='laserscan_kinect_exe',
        #     parameters=[params_file],
        #     remappings=[
        #         ('/image', '/rae/stereo_front/image_raw'),
        #         ('/camera_info', '/rae/stereo_front/camera_info'),
        #     ]
        # ),
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
        DeclareLaunchArgument('params_file', default_value=os.path.join(bringup_prefix, 'config', 'sim.yaml')),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
