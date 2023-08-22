import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, ExecuteProcess, RegisterEventHandler, TimerAction, LogInfo
from launch.event_handlers import  OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.launch_description_sources import PythonLaunchDescriptionSource


def launch_setup(context, *args, **kwargs):
    log_level = 'info'
    if (context.environment.get('DEPTHAI_DEBUG') == '1'):
        log_level = 'debug'
    params_file = LaunchConfiguration("params_file")
    name = LaunchConfiguration('name').perform(context)
    hw_prefix = get_package_share_directory('rae_hw')
    reset_pwm = ExecuteProcess(
        cmd=[['busybox devmem 0x20320180 32 0x00000000']],
        shell=True
    )
    remappings = [
        #   ('imu', '/imu/data'),
        # ('__camera', 'rae'),
        ('camera:__node', 'rae'),
        ('odom', '/diff_controller/odom'),
        ("rgb/image", name+"/right_front/image_raw"),
        ("rgb/camera_info", name+"/right_front/camera_info"),
        ("depth/image", name+"/stereo_front/image_raw"),
        ('image', name+'/right_front/image_raw'),
        ('camera_info', name+'/right_front/camera_info'),
        ('image_rect', name+'/right_front/image_rect')
    ]

    perception = Node(
            package='rae_camera',
            executable='perception_ipc_rtabmap',
            output='screen',
            parameters=[params_file],
            remappings=remappings
           )

    return [
        perception,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(hw_prefix, 'launch', 'control.launch.py'))),
        RegisterEventHandler(
            OnProcessStart(
                target_action=perception,
                on_start=[
                    TimerAction(
                        period=15.0,
                        actions=[reset_pwm, LogInfo(msg='Resetting PWM.'),],
                    )
                ]
            )
        ),
    ]


def generate_launch_description():
    camera_prefix = get_package_share_directory("rae_camera")
    declared_arguments = [
        DeclareLaunchArgument("name", default_value="rae"),
        DeclareLaunchArgument("params_file", default_value=os.path.join(
            camera_prefix, 'config', 'ipc.yaml')),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
