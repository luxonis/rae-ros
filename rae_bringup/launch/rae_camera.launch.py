import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.launch_description_sources import PythonLaunchDescriptionSource


def launch_setup(context, *args, **kwargs):
    log_level = 'info'
    if (context.environment.get('DEPTHAI_DEBUG') == '1'):
        log_level = 'debug'
    params_file = LaunchConfiguration("params_file")
    name = LaunchConfiguration('name').perform(context)

    return [
        ComposableNodeContainer(
            name=name+"_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[
                    ComposableNode(
                        package="depthai_ros_driver",
                        plugin="depthai_ros_driver::Camera",
                        name=name,
                        parameters=[params_file],
                    ),
                    ComposableNode(
                        package="depthai_filters",
                        plugin="depthai_filters::SpatialBB",
                        remappings=[
                                    ('stereo/camera_info', name+'/stereo_front/camera_info'),
                                    ('nn/spatial_detections', name+'/nn/spatial_detections'),
                                    ('rgb/preview/image_raw', name+'/nn/passthrough/image_raw')]
                    ),
                    ComposableNode(
                        package='depthimage_to_laserscan',
                        plugin='depthimage_to_laserscan::DepthImageToLaserScanROS',
                        name='depthimage_to_laserscan_node',
                        remappings=[('depth', name+'/stereo_front/image_raw'),
                                    ('depth_camera_info', name+'/stereo_front/camera_info')],
                        parameters=[{'output_frame': 'base_link',
                                    'range_max': 20.0,
                                    'scan_row': 250}]
         ),
            ],
            arguments=['--ros-args', '--log-level', log_level],
            output="both",
        ),

    ]


def generate_launch_description():
    rae_prefix = get_package_share_directory("rae_bringup")
    declared_arguments = [
        DeclareLaunchArgument("name", default_value="rae"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(
            rae_prefix, 'config', 'camera.yaml')),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
