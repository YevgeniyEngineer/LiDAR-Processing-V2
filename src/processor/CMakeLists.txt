# CMake version
cmake_minimum_required(VERSION 3.18 FATAL_ERROR)

# Project name
project(processor)
include(CTest) # This enables the BUILD_TESTING option

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED)
find_package(PCL 1.7.2 REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_subdirectory(./src/unordered_dense)

# Add executable
add_executable(${PROJECT_NAME}
    src/noise_remover.cpp
    src/segmenter.cpp
    src/clusterer.cpp
    src/polygonizer.cpp
    src/processor.cpp
)

# Include directories for the target
target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${PCL_INCLUDE_DIRS}
        ${rclcpp_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Define dependencies for the target
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    sensor_msgs
    geometry_msgs
    visualization_msgs
    Eigen3
    OpenCV
    PCL
)

# Add PCL and rclcpp libraries to the linker for the target
target_link_libraries(${PROJECT_NAME}
    Eigen3::Eigen
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    ${rclcpp_LIBRARIES}
    unordered_dense
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

# Unit testing
if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    find_package(ament_cmake_gtest REQUIRED)

    # Define the test executable and the source files for the test
    ament_add_gtest(test_containers
        tests/test_circular_deque.cpp
    )

    # Link the test executable
    target_include_directories(test_containers
        PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/src
    )

    # Automatically find and add linters to the test dependencies
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
