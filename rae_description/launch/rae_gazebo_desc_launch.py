from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    bringup_dir = get_package_share_directory('rae_description')
    xacro_path = os.path.join(bringup_dir, 'urdf', 'rae.urdf.xacro')
    sim = False
    if LaunchConfiguration('sim').perform(context):
        sim = True

    rsp_node =  Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': Command(
                [
                    'xacro', ' ', xacro_path, ' ', ' sim_mode:=', LaunchConfiguration('sim').perform(context)
                ]),
                'use_sim_time': sim}]
        )

    joint_state_publisher = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_sim_time': sim}])

    return [rsp_node]
def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("sim", default_value="false"),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )