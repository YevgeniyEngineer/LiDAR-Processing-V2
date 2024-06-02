import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("processor"), "config", "processor.param.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package="processor",
                executable="processor",
                name="processor",
                parameters=[config],
                output="screen",
            )
        ]
    )
