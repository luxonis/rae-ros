import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch.conditions import IfCondition
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):

    params_file = LaunchConfiguration("params_file")
    name = LaunchConfiguration('name').perform(context)
    parent_frame = LaunchConfiguration('parent_frame',  default = 'oak-d-base-frame')
    urdf_launch_dir = os.path.join(get_package_share_directory('depthai_descriptions'), 'launch')
    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    return [
            Node(
                condition=IfCondition(LaunchConfiguration("use_rviz").perform(context)),
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="log",
                arguments=["-d", LaunchConfiguration("rviz_config")],
            ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(urdf_launch_dir, 'urdf_launch.py')),
            launch_arguments={'tf_prefix': name,
                              'base_frame': name,
                              'parent_frame': parent_frame,
                              'use_composition': 'true',
                              'use_base_descr': 'true'}.items()),

        ComposableNodeContainer(
            name=name+"_container",
            namespace="",
            package="rclcpp_components",
            executable="component_container_mt",
            composable_node_descriptions=[
                ComposableNode(
                    package="depthai_ros_driver",
                    plugin="depthai_ros_driver::Camera",
                    name=name,
                    parameters=[params_file],
                ),
                ComposableNode(
                    package='spectacularai_ros2',
                    plugin='spectacularAI::ros2::Node',
                    parameters=[
                        {"imu_frame_id": name+"_imu_frame"},
                        {"world_frame_id": "world"},
                        {"cam0_frame_id": name+"_left_camera_optical_frame"},
                        {"cam1_frame_id": name+"_right_camera_optical_frame"},
                        {"depth_scale": 1.0/1000.0}, # Depth map values are multiplied with this to get distance in meters
                        {"camera_input_type": "stereo_depth_features"},
                        {"recording_folder": LaunchConfiguration('recording_folder').perform(context)},
                        {"enable_mapping": True},
                        {"enable_occupancy_grid": True},
                    ],
                    remappings=[
                        ('input/imu', name + '/imu/data'),
                        ('input/cam0/image_rect', name + '/right/image_rect'),
                        ('input/cam1/image_rect', name + '/left/image_rect'),
                        ('input/cam0/camera_info', name + '/right/camera_info'),
                        ('input/cam1/camera_info', name + '/left/camera_info'),
                        ('input/depth/image', name + '/stereo/image_raw'),
                        ('input/cam0/features', name + '/right_rect_feature_tracker/tracked_features'),
                    ]
                )
            ],
            arguments=['--ros-args', '--log-level', 'info'],
            output="both",
        )
    ]


def generate_launch_description():
    spectacular_prefix = get_package_share_directory("spectacularai_ros2")
    declared_arguments = [
        DeclareLaunchArgument("name", default_value="rae"),
        DeclareLaunchArgument("use_rviz", default_value='false'),
        DeclareLaunchArgument("recording_folder", default_value=""),
        DeclareLaunchArgument("parent_frame", default_value="oak-d-base-frame"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(spectacular_prefix, 'launch', 'oak_d.yaml')),
        DeclareLaunchArgument("rviz_config", default_value=os.path.join(spectacular_prefix, 'launch', 'oak_d.rviz')),
    ]
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )



