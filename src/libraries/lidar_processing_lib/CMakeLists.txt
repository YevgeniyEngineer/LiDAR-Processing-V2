# Set CMake version
cmake_minimum_required(VERSION 3.18 FATAL_ERROR)

# Set project name
project(lidar_processing_lib VERSION 1.0.0 LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find packages
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(PCL 1.7.2 REQUIRED)

# Source files
set(SOURCE_FILES
    src/dummy.cpp
)

# Create shared library
add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})

# Specify include directories for the library
target_include_directories(${PROJECT_NAME}
    PUBLIC
    ${PCL_INCLUDE_DIRS}
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
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
    ${PCL_LIBRARIES}
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
