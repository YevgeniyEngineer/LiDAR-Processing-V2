# cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ..
# cmake --build . --target all --config RelWithDebInfo

cmake_minimum_required(VERSION 3.18 FATAL_ERROR)

project(lidar_processing_v2)

find_package(ament_cmake REQUIRED)

add_subdirectory(ros2_nodes/dataloader)

ament_package()
