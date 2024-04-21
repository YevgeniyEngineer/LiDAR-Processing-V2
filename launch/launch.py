import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    # RViz
    package_directory = os.path.join(
        get_package_share_directory("lidar_processing_v2"), "visualization"
    )
    config_file = os.path.join(package_directory, "rviz2_config.rviz")
    rviz = ExecuteProcess(cmd=["rviz2", "-d", config_file], output="screen")

    # Sensor Data Publisher Node
    package_directory = get_package_share_directory("dataloader")
    config_file = os.path.join(package_directory, "config", "dataloader.param.yaml")
    dataloader = Node(
        package="dataloader",
        executable="dataloader",
        name="dataloader",
        output="screen",
        parameters=[config_file],
    )

    # Create and return the launch description
    return LaunchDescription([rviz, dataloader])
