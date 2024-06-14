cmake_minimum_required(VERSION 3.18 FATAL_ERROR)
project(dataloader)

# Set C++17 Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(PCL 1.7.2 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Add executable
add_executable(${PROJECT_NAME}
    src/dataloader.cpp
)

# Include directories for the target
target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${PCL_INCLUDE_DIRS}
        ${rclcpp_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Define dependencies for the target
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    sensor_msgs
    visualization_msgs
)

# Add PCL and rclcpp libraries to the linker for the target
target_link_libraries(${PROJECT_NAME}
    ${PCL_LIBRARIES}
    ${rclcpp_LIBRARIES}
)

# Add PCL definitions to the target
target_compile_definitions(${PROJECT_NAME}
    PRIVATE
        ${PCL_DEFINITIONS}
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
