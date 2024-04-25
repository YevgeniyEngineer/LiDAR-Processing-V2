cmake_minimum_required(VERSION 3.18 FATAL_ERROR)
project(launch_pkg)

find_package(ament_cmake REQUIRED)

install(FILES launch/global_launch.py
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
