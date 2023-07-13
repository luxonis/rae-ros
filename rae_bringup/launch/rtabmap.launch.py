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
    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    name = LaunchConfiguration('name').perform(context)

    parameters = [
        {
            "frame_id": "base_footprint",
            "subscribe_rgb": True,
            "subscribe_depth": True,
            "subscribe_odom_info": False,
            "approx_sync": True,
            "Rtabmap/DetectionRate": "3.5",
        }
    ]

    remappings = [
        #   ('imu', '/imu/data'),
        ("rgb/image", name+"/right_front/image_rect"),
        ("rgb/camera_info", name+"/right_front/camera_info"),
        ("depth/image", name+"/stereo_front/image_raw"),
    ]

    return [


        LoadComposableNodes(
            target_container=name+"_container",
            composable_node_descriptions=[
                ComposableNode(
                    package="image_proc",
                    plugin="image_proc::RectifyNode",
                    name="rectify_color_node",
                    remappings=[('image', name+'/right_front/image_raw'),
                                ('camera_info', name+'/right_front/camera_info'),
                                ('image_rect', name+'/right_front/image_rect'),]
                )
            ]),
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']),
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']),

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
