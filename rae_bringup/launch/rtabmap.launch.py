import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import  Node,LoadComposableNodes
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
            "subscribe_depth": True,
            "approx_sync": True,
            "approx_sync_max_interval":0.001,
            "Rtabmap/DetectionRate": "1",
            #"odom_topic": "/odom",
            "qos": 1,
            "visual_odometry": True,
            "queue_size":100,
            "Rtabmap/TimeThr": "700",
            "Kp/WordsPerImage": "200",
            "Kp/RoiRatios": "0.03 0.03 0.04 0.04",
            "Kp/DetectorStrategy": "0",
            "SURF/HessianThreshold": "1000",
            "Kp/NNStrategy": "1",
            "LccBow/MinInliers":"10",
            "LccBow/EstimationType":"1",
	     "LccReextract/Activated":"true",
            "LccReextract/MaxWords":"500",
            "LccReextract/MaxDepth":"10",
            "RGBD/OptimizeSlam2D" : True,
            "RGBD/OptimizeStrategy" : 1,
            "RGBD/OptimizeRobust": True,
            "RGBD/OptimizeMaxError": "0",
        }
    ]

    remappings = [
        #   ('imu', '/imu/data'),
        #('odom', '/odometry/filtered'),
        ('rgb/image', 'image_rect'),
        ('rgb/camera_info', name+'/right/camera_info'),
        ('depth/image', name+'/stereo/image_raw'),
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
                                #('image_rect', name+'/right/image_rect'),
                                ]
                ),
                ComposableNode(
                    package='rtabmap_slam',
                    plugin='rtabmap_slam::CoreWrapper',
                    parameters=parameters,
                    remappings=remappings,
                    )
            ]),
         Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=parameters,
            remappings=remappings),

        # LoadComposableNodes(
        #     target_container=name+'_container',
        #     composable_node_descriptions=[
        #             ComposableNode(
        #                 package='laserscan_kinect',
        #                 plugin='laserscan_kinect::LaserScanKinectNode',
        #                 name='laserscan_kinect_front',
        #                 parameters=[laserscan_config],
        #                 remappings=[
        #                     ('/image', name+'/stereo/image_raw'),
        #                     ('/camera_info', name+'/stereo/camera_info'),
        #                    #('/scan', name+'/scan'),
        #                    ('/debug_image', name+'/debug'),
        #                    ('/debug_image/compressed', name+'/debug_image/compressed')
        #                 ]
        #             )
                   
        #     ]
        # )  
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

