#!/bin/bash

# Absolute path to the root of the project
ROOT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )/.." &> /dev/null && pwd )

# Unset GTK_PATH needed for Rviz to avoid crashes
unset GTK_PATH

# For ROS line buffering
export RCUTILS_LOGGING_USE_STDOUT=1  # Enable logging to stdout
export RCUTILS_LOGGING_BUFFERED_STREAM=1  # Enable buffering
export QT_X11_NO_MITSHM=1 # Disable the use of MIT-SHM (shared memory extension)

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

# Default package and launch file
DEFAULT_PACKAGE="launch_pkg"
DEFAULT_LAUNCH_FILE="global_launch.py"

# Specific configuration for RVIZ2
RVIZ2_CONFIG_PATH="${ROOT_DIR}/src/rviz2_config/config/rviz2_config.rviz"

# Check if the first argument is 'rviz2'
if [[ "$1" == "rviz2" ]]; then
    echo "Detected request to launch rviz2 using config: $RVIZ2_CONFIG_PATH"
    ros2 run rviz2 rviz2 --display-config "$RVIZ2_CONFIG_PATH"
    exit 0
elif [ ! -z "$1" ]; then
    # Automatically derive the launch file name from the package name
    PACKAGE_NAME="$1"
    LAUNCH_FILE_NAME="${PACKAGE_NAME}.launch.py"
    PACKAGE_TO_LAUNCH="$PACKAGE_NAME $LAUNCH_FILE_NAME"
    echo "Launching $PACKAGE_NAME with $LAUNCH_FILE_NAME"
    ros2 launch $PACKAGE_TO_LAUNCH
else
    # Launch the default package and launch file
    PACKAGE_TO_LAUNCH="$DEFAULT_PACKAGE $DEFAULT_LAUNCH_FILE"
    echo "Launching default package with: ros2 launch $PACKAGE_TO_LAUNCH"
    ros2 launch $PACKAGE_TO_LAUNCH
fi
