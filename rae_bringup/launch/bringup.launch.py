import os
from ament_index_python.packages import get_package_share_path
import launch
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    bringup_prefix = get_package_share_path('rae_bringup')
    DeclareLaunchArgument('use_sim_time', default_value='True')
    LaunchConfiguration('use_sim_time', default='True')
    params = os.path.join(bringup_prefix, 'config', 'sim.yaml')
    rviz_config = os.path.join(bringup_prefix, "config", "sim.rviz")
    return launch.LaunchDescription([
        IncludeLaunchDescription(os.path.join(get_package_share_path('rae_gazebo'), 'launch', 'rae_simulation.launch.py')),
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan_node',
            remappings=[('depth', '/camera/depth_image'),
                        ('depth_camera_info', '/camera/camera_info')],
            parameters=[{'output_frame': 'rgb_camera_link_optical_frame',
                        'range_max': 20.0}]
        ),
        # Node(
        #         package="rviz2",
        #         executable="rviz2",
        #         name="rviz2",
        #         output="log",
        #         arguments=["-d", rviz_config],
        #         parameters=[{'use_sim_time': True}]
        #     ),
        # IncludeLaunchDescription(os.path.join(get_package_share_path('nav2_bringup'), 'launch', 'slam_launch.py'),
        # launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time'),
        # 'params_file': params}.items()),
        IncludeLaunchDescription(os.path.join(bringup_prefix, 'launch', 'nav.launch.py'),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time'),
        'params_file': params}.items())
    ])