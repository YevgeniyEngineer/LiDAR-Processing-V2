cmake_minimum_required(VERSION 3.18 FATAL_ERROR)
project(rviz2_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
