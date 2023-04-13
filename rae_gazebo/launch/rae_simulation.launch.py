import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    pkg_rae_description = get_package_share_directory(
        'rae_description')
    world_file = LaunchConfiguration('sdf_file').perform(context)
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')


    return [
        SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            str(Path(pkg_rae_description).parent.resolve())]),
       IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': world_file,
            'use_sim_time': 'True'
        }.items(),
        
    ),
    IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('rae_description'), 'launch', 'rae_gazebo_desc_launch.py')),
      launch_arguments={"sim": "true"}.items()
      ),
    Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan_node',
            remappings=[('depth', '/camera/depth_image'),
                        ('depth_camera_info', '/camera/camera_info')],
            parameters=[{'output_frame': 'rgb_camera_link_optical_frame',
                        'range_max': 20.0}]
        ),
    Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description', '-z', '0.15'
                   ],
                   parameters=[{'use_sim_time': True}],
        output='screen'
    ),
    
    Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
                    '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                    '/camera/image@sensor_msgs/msg/Image@ignition.msgs.Image',
                    '/camera/depth_image@sensor_msgs/msg/Image@ignition.msgs.Image',
                    '/camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
                    '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
                    '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                    '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'
                   ],
                   parameters=[{'use_sim_time': True}],
        output='screen'
    )
    ]


def generate_launch_description():
    rae_gazebo_package = get_package_share_directory("rae_gazebo")
    declared_arguments = [
        DeclareLaunchArgument("sdf_file", default_value=f"-r {os.path.join(rae_gazebo_package, 'world', 'robot_demo.sdf')}"),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
