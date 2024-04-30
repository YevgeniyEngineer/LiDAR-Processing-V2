# Set CMake version
cmake_minimum_required(VERSION 3.18 FATAL_ERROR)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set global project name
project(lidar_processing_v2)

# Add subdirectories
add_subdirectory(../src/libraries/containers containers)
add_subdirectory(../src/libraries/lidar_processing_lib lidar_processing_lib)
