# CMake version
cmake_minimum_required(VERSION 3.18 FATAL_ERROR)

# Project name
project(processor)
include(CTest) # This enables the BUILD_TESTING option

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable profiling
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")
# set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg")

# Find required packages
find_package(lidar_processing_lib REQUIRED)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED)
find_package(PCL 1.7.2 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Add executable
add_executable(${PROJECT_NAME}
    src/processor.cpp
)

# Set compiler flags
target_compile_options(${PROJECT_NAME}
    PRIVATE
        "-Wall"    # Enable all common warnings
        "-Wextra"  # Enable extra warnings
        "-Werror"  # Treat warnings as errors
)

# Include directories for the target
target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${PCL_INCLUDE_DIRS}
        ${rclcpp_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Link libraries
target_link_libraries(${PROJECT_NAME}
    lidar_processing_lib
)

# Add PCL and rclcpp libraries to the linker for the target
target_link_libraries(${PROJECT_NAME}
    lidar_processing_lib
    Eigen3::Eigen
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    ${rclcpp_LIBRARIES}
)

# Define dependencies for the target
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    sensor_msgs
    geometry_msgs
    visualization_msgs
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
