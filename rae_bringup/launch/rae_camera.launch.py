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
    if(context.environment.get('DEPTHAI_DEBUG')=='1'):
        log_level='debug'
    params_file = LaunchConfiguration("params_file")
    depthai_prefix = get_package_share_directory("depthai_ros_driver")
    name = LaunchConfiguration('name').perform(context)

    parameters = [
        {
            "frame_id": "base_footprint",
            "subscribe_rgb": True,
            "subscribe_depth": True,
            "subscribe_odom_info": True,
            "approx_sync": True,
            "Rtabmap/DetectionRate": "3.5",
        }
    ]

    remappings=[
        #   ('imu', '/imu/data'),
        ("rgb/image", name+"/right_front/image_rect"),
        ("rgb/camera_info", name+"/right_front/camera_info"),
        ("depth/image", name+"/stereo_front/image_raw"),
    ]

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
        Node(
            package='laserscan_kinect',
            executable='laserscan_kinect_exe',
            parameters=[params_file],
            remappings=[
                ('/image', '/rae/stereo_front/image_raw'),
                ('/camera_info', '/rae/stereo_front/camera_info'),
            ]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('rae_description'), 'launch', 'rsp.launch.py')),
            launch_arguments={"sim": "false"}.items()
        ),
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
                    # ComposableNode(
                    # package='depth_image_proc',
                    # plugin='depth_image_proc::PointCloudXyzrgbNode',
                    # name='point_cloud_xyzrgb_node',
                    # remappings=[('depth_registered/image_rect', name+'/stereo_front/image_raw'),
                    #             ('rgb/image_rect_color', name+'/right_front/image_rect'),
                    #             ('rgb/camera_info', name+'/right_front/camera_info'),
                    #             ('points', name+'/points')]
                    # ),
            ],
            arguments=['--ros-args', '--log-level', log_level],
            output="both",
        ), 
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

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=parameters,
            remappings=remappings),
        # LoadComposableNodes(
        #     target_container=name+"_container",
        #     composable_node_descriptions=[
        #         ComposableNode(
        #             package='rtabmap_ros',
        #             plugin='rtabmap_ros::RGBDOdometry',
        #             name='rgbd_odometry',
        #             parameters=parameters,
        #             remappings=remappings,
        #         ),
        #     ],
        # ),

        # LoadComposableNodes(
        #     target_container=name+"_container",
        #     composable_node_descriptions=[
        #         ComposableNode(
        #             package='rtabmap_ros',
        #             plugin='rtabmap_ros::CoreWrapper',
        #             name='rtabmap',
        #             parameters=parameters,
        #             remappings=remappings,
        #         ),
        #     ],
        # ),

        # Node(
        #     package="rtabmap_ros",
        #     executable="rtabmapviz",
        #     output="screen",
        #     parameters=parameters,
        #     remappings=remappings,
        # ),
    ]


def generate_launch_description():
    rae_prefix = get_package_share_directory("rae_bringup")
    declared_arguments = [
        DeclareLaunchArgument("name", default_value="rae"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(rae_prefix, 'config', 'camera.yaml')),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )