import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
import xacro


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="rae_hw",
            executable="battery",
        ),
        Node(
            package="rae_hw",
            executable="lcd",
        ),
        Node(
            package="rae_hw",
            executable="led",
            output="screen",
        ),
        Node(
            package="rae_hw",
            executable="mic",
        ),
        Node(
            package="rae_hw",
            executable="speakers",
        ),
        Node(
            package="rae_bringup",
            executable="led_test.py"
        ),
    ])