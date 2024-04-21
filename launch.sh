#!/bin/bash

# Re-build first
# colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# Location of script's directory
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

unset GTK_PATH  # Needed for Rviz, else crashes

ROS_SETUP_SCRIPT="/opt/ros/humble/setup.bash"

if [ -f "$ROS_SETUP_SCRIPT" ]; then
    echo "Sourcing $ROS_SETUP_SCRIPT"
    source "$ROS_SETUP_SCRIPT"
else
    echo "Warning: Unable to source $ROS_SETUP_SCRIPT, file does not exist."
    exit 1
fi

LOCAL_SETUP_SCRIPT="${SCRIPT_DIR}/install/setup.bash"

if [ -f "$LOCAL_SETUP_SCRIPT" ]; then
    echo "Sourcing $LOCAL_SETUP_SCRIPT"
    source "$LOCAL_SETUP_SCRIPT"
else
    echo "Warning: Unable to source $LOCAL_SETUP_SCRIPT, file does not exist."
    exit 1
fi

ros2 launch lidar_processing_v2 launch.py
