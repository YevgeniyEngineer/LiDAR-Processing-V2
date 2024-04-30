# Set CMake version
cmake_minimum_required(VERSION 3.18 FATAL_ERROR)

# Set project name
project(containers VERSION 0.0.0 LANGUAGES CXX)
include(CTest) # This enables the BUILD_TESTING option

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find packages
find_package(ament_cmake QUIET)

if (ament_cmake_FOUND)
    message(STATUS "${PROJECT_NAME}: ament_cmake found")
else()
    message(STATUS "${PROJECT_NAME}: ament_cmake not found")
endif()

# Create shared library
add_library(${PROJECT_NAME} INTERFACE)

# Specify include directories for the library
target_include_directories(${PROJECT_NAME}
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Installation rules for the library
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
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
    if(ament_cmake_FOUND)
        find_package(ament_lint_auto REQUIRED)
        find_package(ament_cmake_gtest REQUIRED)

        # Define the test executable and the source files for the test
        ament_add_gtest(test_containers
            tests/test_queue.cpp
            tests/test_vector.cpp
        )

        # Link the test executable
        target_link_libraries(test_containers
            ${PROJECT_NAME}
        )

        # Automatically find and add linters to the test dependencies
        ament_lint_auto_find_test_dependencies()
    else()
        # Non-ROS2 environment, use GoogleTest directly
        find_package(GTest REQUIRED)

        # Define the test executable and the source files for the test
        add_executable(test_containers
            tests/test_queue.cpp
            tests/test_vector.cpp
        )

        target_link_libraries(test_containers
            ${PROJECT_NAME}
            GTest::GTest
            GTest::Main
        )

        enable_testing()
        add_test(NAME test_containers COMMAND test_containers)
    endif()
endif()

if (ament_cmake_FOUND)
    ament_package()
endif()
