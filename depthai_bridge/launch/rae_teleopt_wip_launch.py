import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    depthai_bridge = get_package_share_directory('depthai_bridge')

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'ign_args': '-r robot_demo.sdf'
        }.items(),
    )

    # worth thinking about moving base link to a midpoint between wheels for easier/more intuitive tfs
    publish_robot_info = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(depthai_bridge, 'launch', 'rae_gazebo_desc_launch.py')),
    )
    #TODO Danilo  - launch file expects to be called inside src/depthai-ros folder in order to find meshes - either hardcore where meshes are or find different solution 
    spawn_robot=Node(
        package='ros_ign_gazebo',
        executable='create',
        arguments=['-topic', '/robot_description', '-z', '0.15'
                   ],
        output='screen'
    )

    

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                   #'/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry'
                   ],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        publish_robot_info,
        spawn_robot,
        bridge

    ])
