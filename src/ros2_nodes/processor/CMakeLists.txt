# CMake version
cmake_minimum_required(VERSION 3.18 FATAL_ERROR)

# Project name
project(processor)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(lidar_processing_lib REQUIRED)

# Add executable
add_executable(${PROJECT_NAME}
    src/processor.cpp
)

# Include directories for the target
target_include_directories(${PROJECT_NAME}
    PRIVATE
        include
)

# Define dependencies for the target
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    sensor_msgs
    geometry_msgs
    visualization_msgs
    lidar_processing_lib
)

# Install the executable (ROS2 convention)
install(TARGETS ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
)

# Install the config
install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

# Install the launch
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
