#!/bin/bash

# Absolute path to the root of the project
ROOT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )/.." &> /dev/null && pwd )

# Re-build first
echo "Building the project..."
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release

# Unset GTK_PATH needed for Rviz to avoid crashes
unset GTK_PATH

# For ROS line buffering
export RCUTILS_LOGGING_USE_STDOUT=1  # Enable logging to stdout
export RCUTILS_LOGGING_BUFFERED_STREAM=1  # Enable buffering

# ROS setup script
ROS_SETUP_SCRIPT="/opt/ros/humble/setup.bash"

if [ -f "$ROS_SETUP_SCRIPT" ]; then
    echo "Sourcing $ROS_SETUP_SCRIPT"
    source "$ROS_SETUP_SCRIPT"
else
    echo "Warning: Unable to source $ROS_SETUP_SCRIPT, file does not exist."
    exit 1
fi

# Local setup script
LOCAL_SETUP_SCRIPT="${ROOT_DIR}/install/local_setup.bash"

if [ -f "$LOCAL_SETUP_SCRIPT" ]; then
    echo "Sourcing $LOCAL_SETUP_SCRIPT"
    source "$LOCAL_SETUP_SCRIPT"
else
    echo "Warning: Unable to source $LOCAL_SETUP_SCRIPT, file does not exist."
    exit 1
fi

# Launch ROS2 nodes using the launch file
echo "Launching ROS2 nodes..."
ros2 launch launch_pkg global_launch.py
