import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    world_file = LaunchConfiguration('sdf_file').perform(context)
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    return [
       IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'ign_args': world_file
        }.items(),
    ),
    Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                   #'/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry'
                   ],
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