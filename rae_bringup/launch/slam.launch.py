import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.launch_description_sources import PythonLaunchDescriptionSource



def launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration("params_file")
    rae_prefix = get_package_share_directory('rae_bringup')
    name = LaunchConfiguration('name').perform(context)
    laserscan_config = os.path.join(
        get_package_share_directory('laserscan_kinect'),
        'config',
        'params.yaml'
    )
    return [
        Node(
        parameters=[
          params_file,
          {'use_sim_time': False}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'),
        LoadComposableNodes(
            target_container=name+"_container",
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
                        package="ira_laser_tools",
                        name="laser_scan_multi_merger",
                        plugin="ira_laser_tools::LaserscanMerger",
                        parameters=[{'laserscan_topics': '/rae/scan_back /rae/scan_front',
                                    'destination_frame': 'base_link',
                                    'scan_destination_topic': '/scan'}
                                    ]
        ),
            ]
        )  
            
    ]
def generate_launch_description():
    bringup_prefix = get_package_share_directory('rae_bringup')
    rviz_config = os.path.join(bringup_prefix, "config", "sim.rviz")
    declared_arguments = [
        DeclareLaunchArgument("name", default_value="rae"),
        DeclareLaunchArgument('sim`', default_value='False'),
        DeclareLaunchArgument('use_rviz', default_value='False'),
        DeclareLaunchArgument('rviz_config', default_value=rviz_config),
        DeclareLaunchArgument('params_file', default_value=os.path.join(bringup_prefix, 'config', 'slam_param.yaml')),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
