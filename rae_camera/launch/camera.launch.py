from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            package="rae_camera",
            executable="camera",
            parameters=[{'enable_rgb': False}]
        ),
    ])