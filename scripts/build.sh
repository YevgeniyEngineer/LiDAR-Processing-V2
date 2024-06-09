#!/bin/bash

# Function to display help message
function display_help() {
    echo "Usage: $0 [-d package1,package2,...] [-h]"
    echo "  -d  Specify packages to build in Debug mode"
    echo "  -h  Display this help message"
    echo "  Builds all other packages in Release mode by default."
}

# Initialize variables
DEBUG_PACKAGES=()
RELEASE_ONLY=true

# Parse command line options
while getopts ":d:h" opt; do
    case "$opt" in
        d)
            # Replace commas with spaces and then read into array
            ARG=${OPTARG//,/ }
            IFS=' ' read -ra DEBUG_PACKAGES <<< "$ARG"
            RELEASE_ONLY=false
            ;;
        h)
            display_help
            exit 0
            ;;
        \?)
            echo "Invalid option: -$OPTARG" >&2
            display_help
            exit 1
            ;;
    esac
done

# Build debug packages if specified
if [ "${#DEBUG_PACKAGES[@]}" -gt 0 ]; then
    echo "Building the following packages in Debug mode: ${DEBUG_PACKAGES[*]}"
    colcon build --packages-select "${DEBUG_PACKAGES[@]}" \
                 --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Debug
fi

# Build all other packages in Release mode
if [ "$RELEASE_ONLY" = true ]; then
    echo "Building all packages in Release mode"
    colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release
else
    # Exclude debug packages when building the rest in Release mode
    echo "Building remaining packages in Release mode"
    colcon build --packages-skip "${DEBUG_PACKAGES[@]}" \
                 --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release
fi
