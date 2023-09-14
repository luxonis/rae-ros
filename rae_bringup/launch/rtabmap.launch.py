import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import  LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):

    rae_prefix = get_package_share_directory('rae_camera')
    name = LaunchConfiguration('name').perform(context)
    laserscan_config = os.path.join(
        rae_prefix,
        'config',
        'laserscan_kinect.yaml'
    )
    parameters = [
        {
            'frame_id': 'base_footprint',
            'subscribe_rgb': True,
            'subscribe_depth': True,
            'subscribe_scan': True,
            'subscribe_odom_info': False,
            'approx_sync': True,
            'Grid/MaxGroundHeight': '0.1',
            'Grid/FromDepth': True,
            'Grid/RangeMin': '0.4',
            'Grid/RangeMax': '8.0',
            'Kp/RoiRatio': '0 0 0 0.3',
            # 'Grid/3D': 'false',
            'Grid/MaxGroundAngle': '60.0',
            'Grid/FootprintHeight': '0.1',
            'Reg/Force3DoF': 'true',
            'Optimizer/Slam2D': True,
            'Rtabmap/DetectionRate': '1.0',
            'Grid/RayTracing': 'true',
            'RGBD/NeighborLinkRefining':'True',
            'RGBD/LocalLoopDetectionTime': 'false',
            'RGBD/OptimizeFromGraphEnd': 'false',
            'RGBD/AngularUpdate': '0.01',
            'RGBD/LinearUpdate': '0.01',
#            'Reg/Strategy': '1',
            'qos_scan': 2
        }
    ]

    remappings = [
        #   ('imu', '/imu/data'),
        ('odom', '/diff_controller/odom'),
        ('rgb/image', name+'/right/image_raw'),
        ('rgb/camera_info', name+'/right/camera_info'),
        ('depth/image', name+'/stereo_front/image_raw'),
    ]

    return [


        LoadComposableNodes(
            target_container=name+'_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='image_proc',
                    plugin='image_proc::RectifyNode',
                    name='rectify_color_node',
                    remappings=[('image', name+'/right/image_raw'),
                                ('camera_info', name+'/right/camera_info'),
                                ('image_rect', name+'/right/image_rect'),]
                ),
                ComposableNode(
                    package='rtabmap_slam',
                    plugin='rtabmap_slam::CoreWrapper',
                    parameters=parameters,
                    remappings=remappings,
                    )
            ]),

        LoadComposableNodes(
            target_container=name+'_container',
            composable_node_descriptions=[
                    ComposableNode(
                        package='laserscan_kinect',
                        plugin='laserscan_kinect::LaserScanKinectNode',
                        name='laserscan_kinect_front',
                        parameters=[laserscan_config],
                        remappings=[
                            ('/image', name+'/stereo_front/image_raw'),
                            ('/camera_info', name+'/stereo_front/camera_info'),
                            ('/scan', name+'/scan_front'),
                            ('/debug_image', name+'/debug_image_front'),
                            ('/debug_image/compressed', name+'/debug_image_front/compressed')
                        ]
                    ),
                    ComposableNode(
                        package='laserscan_kinect',
                        plugin='laserscan_kinect::LaserScanKinectNode',
                        name='laserscan_kinect_back',
                        parameters=[laserscan_config],
                        remappings=[
                            ('/image', name+'/stereo_back/image_raw'),
                            ('/camera_info', name+'/stereo_back/camera_info'),
                            ('/scan', name+'/scan_back'),
                            ('/debug_image', name+'/debug_image_back'),
                            ('/debug_image/compressed', name+'/debug_image_back/compressed')
                        ]
                    ),
                    ComposableNode(
                        package='ira_laser_tools',
                        name='laser_scan_multi_merger',
                        plugin='ira_laser_tools::LaserscanMerger',
                        parameters=[{'laserscan_topics': '/rae/scan_back /rae/scan_front',
                                    'destination_frame': 'base_link',
                                    'scan_destination_topic': '/scan'}
                                    ]
        ),
            ]
        )  
        ]


def generate_launch_description():
    rae_prefix = get_package_share_directory('rae_bringup')
    declared_arguments = [
        DeclareLaunchArgument('name', default_value='rae'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            rae_prefix, 'config', 'camera.yaml')),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
