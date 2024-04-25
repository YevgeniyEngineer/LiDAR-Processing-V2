# Set CMake version
cmake_minimum_required(VERSION 3.18 FATAL_ERROR)

# Set project name
project(lidar_processing_lib VERSION 1.0.0 LANGUAGES CXX)
# include(CTest) # This enables the BUILD_TESTING option

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find packages
find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED) # See https://github.com/ros2/eigen3_cmake_module/tree/humble
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED)
find_package(PCL 1.7.2 REQUIRED)

# Source files
set(SOURCES
    src/segmentation/jcp_segmenter.cpp
)

# Create shared library
add_library(${PROJECT_NAME} SHARED ${SOURCES})

# Specify include directories for the library
target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Link directories
target_link_directories(${PROJECT_NAME}
    PUBLIC
        ${PCL_LIBRARY_DIRS}
)

# Compile definitions
target_compile_definitions(${PROJECT_NAME}
    PUBLIC
        ${PCL_DEFINITIONS}
)

# Link libraries
target_link_libraries(${PROJECT_NAME}
    PUBLIC
        Eigen3::Eigen
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
)

# Ament and ROS2 specific configuration
ament_target_dependencies(${PROJECT_NAME}
    PUBLIC
        Eigen3
        OpenCV
        PCL
)

# Export dependencies
ament_export_dependencies(
    eigen3_cmake_module
    Eigen3
    OpenCV
    PCL
)

# Installation rules for the library
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

# Install the headers
install(DIRECTORY include/ DESTINATION include)

# Export the targets
install(EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION lib/cmake/${PROJECT_NAME}
)

# Create a ConfigVersion.cmake file
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    ${PROJECT_NAME}ConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

# Install the Config and ConfigVersion files
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    DESTINATION lib/cmake/${PROJECT_NAME}
)

# Generate and install the Config file
configure_package_config_file(${CMAKE_CURRENT_SOURCE_DIR}/cmake/Config.cmake.in
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
)
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    DESTINATION lib/cmake/${PROJECT_NAME}
)

# Unit testing
if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    find_package(ament_cmake_gtest REQUIRED)

    # Define the test executable and the source file for the test
    # ament_add_gtest(test_segmenter tests/test_segmenter.cpp)

    # Link the test executable
    # target_link_libraries(test_segmenter
    #     ${PROJECT_NAME}
    # )

    # Automatically find and add linters to the test dependencies
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
