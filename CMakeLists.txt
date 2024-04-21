# cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ..
# cmake --build . --target all --config RelWithDebInfo

cmake_minimum_required(VERSION 3.18 FATAL_ERROR)

project(lidar_processing_v2)

set(BUILD_TESTS ON)

find_package(ament_cmake REQUIRED)

add_subdirectory(libraries/utilities_lib)
add_subdirectory(ros2_nodes/dataloader)

# Install the launch
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

# Install the visualization
install(DIRECTORY visualization
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
