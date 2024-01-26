import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    rae_description_pkg = get_package_share_directory('rae_description')
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')
    world_file = LaunchConfiguration('sdf_file').perform(context)
    print(f'world_file: {world_file}')
    return [
        SetEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=[
                str(Path(rae_description_pkg).parent.resolve())
            ]
        ),

        # Ignition Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
            ]),
            launch_arguments={
                'gz_args': world_file,
                'use_sim_time': 'True'
            }.items()
        ),

        # RSP Node
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(rae_description_pkg, 'launch', 'rsp.launch.py')
            ]),
            launch_arguments={
                'use_sim_time': 'True'
            }.items()
        ),

        # ros_gz_bridge
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                '/camera/image@sensor_msgs/msg/Image@ignition.msgs.Image',
                '/camera/depth_image@sensor_msgs/msg/Image@ignition.msgs.Image',
                '/camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',

                '/image_out@sensor_msgs/msg/Image@ignition.msgs.Image',
                '/image_tuning@sensor_msgs/msg/Image@ignition.msgs.Image',

                '/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
                '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
                '/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            ],
            parameters=[{
                'use_sim_time': True
            }],
            output='screen'
        ),

        # Ignition Gazebo - Spawn Entity
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', 'robot_description',
            ],
            output='screen'
        )
    ]



def generate_launch_description():
    rae_gazebo_pkg = get_package_share_directory('rae_gazebo')
    declared_arguments = [
        DeclareLaunchArgument('sdf_file', default_value=f"-r {os.path.join(rae_gazebo_pkg, 'worlds', 'world_demo.sdf')}"),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )