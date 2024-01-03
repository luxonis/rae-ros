import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
import xacro


def launch_setup(context, *args, **kwargs):
    rae_description_path = os.path.join(get_package_share_directory(
        'rae_description'), 'urdf', 'rae.urdf.xacro')
    rae_description_config = xacro.process_file(rae_description_path)
    robot_description = {'robot_description': rae_description_config.toxml()}
    controller_params = LaunchConfiguration('controller_params_file').perform(context)
    run_container = LaunchConfiguration('run_container', default='true')

    enable_localization = LaunchConfiguration(
        'enable_localization', default='true')
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('rae_description'), 'launch', 'rsp.launch.py')),
            launch_arguments={'sim': 'false',
                              'namespace': LaunchConfiguration('namespace'), 
                              }.items()
        ),
        Node(
            condition=IfCondition(enable_localization),
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[os.path.join(get_package_share_directory(
                'rae_hw'), 'config', 'ekf.yaml')],
        ),
        Node(
            package='imu_complementary_filter',
            executable='complementary_filter_node',
            name='complementary_filter_gain_node',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[
                    {'do_bias_estimation': True},
                    {'do_adaptive_gain': True},
                    {'orientation_stddev': 0.001},
                    {'use_mag': False},
                    {'gain_acc': 0.04},
                    {'gain_mag': 0.01},
            ],
            remappings=[
                ('imu/data_raw', '/rae/imu/data'),
            ]
        ),
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            name='controller_manager',
            namespace=LaunchConfiguration('namespace'),
            parameters=[robot_description, controller_params],
            remappings=[('/diff_controller/cmd_vel_unstamped', 'cmd_vel')],
            output={
                'stdout': 'screen',
                'stderr': 'screen',
            },
        ),

        Node(
            package='controller_manager',
            namespace=LaunchConfiguration('namespace'),
            executable='spawner',
            arguments=['diff_controller', '-c', '/controller_manager'],
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            namespace=LaunchConfiguration('namespace'),
            arguments=['joint_state_broadcaster',
                       '--controller-manager', '/controller_manager'],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('rae_hw'), 'launch', 'peripherals.launch.py')),
            launch_arguments={'run_container': run_container,
                              'namespace': LaunchConfiguration('namespace'),
                              }.items()
        ),
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument('name', default_value='rae'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('run_container', default_value='true'),
        DeclareLaunchArgument('enable_battery_status', default_value='true'),
        DeclareLaunchArgument('enable_localization', default_value='true'),
        DeclareLaunchArgument('controller_params_file', default_value=os.path.join(get_package_share_directory(
        'rae_hw'), 'config', 'controller.yaml')),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
