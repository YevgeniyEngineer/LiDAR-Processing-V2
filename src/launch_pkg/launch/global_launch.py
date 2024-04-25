import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the share directory for each package
    pkg_share_dataloader = get_package_share_directory("dataloader")
    pkg_share_processor = get_package_share_directory("processor")
    pkg_rviz2_config = get_package_share_directory("rviz2_config")

    # Create the launch configuration variables
    dataloader_launch_file = os.path.join(
        pkg_share_dataloader, "launch", "dataloader.launch.py"
    )
    dataloader_param_file = os.path.join(
        pkg_share_dataloader, "config", "dataloader.param.yaml"
    )

    processor_launch_file = os.path.join(
        pkg_share_processor, "launch", "processor.launch.py"
    )
    processor_param_file = os.path.join(
        pkg_share_processor, "config", "processor.param.yaml"
    )

    rviz2_config_file = os.path.join(pkg_rviz2_config, "config", "rviz2_config.rviz")

    # Include the individual launch files
    dataloader_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([dataloader_launch_file]),
        launch_arguments={"params_file": dataloader_param_file}.items(),
    )
    processor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([processor_launch_file]),
        launch_arguments={"params_file": processor_param_file}.items(),
    )
    rviz2 = ExecuteProcess(cmd=["rviz2", "-d", rviz2_config_file], output="screen")

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(dataloader_launch)
    # ld.add_action(processor_launch)
    ld.add_action(rviz2)

    return ld
