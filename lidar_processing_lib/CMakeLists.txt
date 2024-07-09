# Specify minimum required CMake version
cmake_minimum_required(VERSION 3.18 FATAL_ERROR)

# Set project name
project(lidar_processing_lib VERSION 1.0.0)

# To enable testing
include(CTest)

# Define common compiler flags
set(COMMON_FLAGS
    "-Wall"                         # Enable all common warnings
    "-Wextra"                       # Enable extra warnings
    "-Werror"                       # Treat warnings as errors
)

# Define debug compiler flags
set(DEBUG_FLAGS
    "-g"                            # Generate debug information
    "-O0"                           # No optimization
)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required dependencies
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED)
find_package(PCL 1.7.2 REQUIRED)

# Set library and sources
add_library(${PROJECT_NAME} SHARED
    src/segmenter.cpp
    src/clusterer.cpp
    src/noise_remover.cpp
    src/polygonizer.cpp
)

# Set library properties
set_target_properties(${PROJECT_NAME} PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)

# Set compiler flags
target_compile_options(${PROJECT_NAME} PRIVATE ${COMMON_FLAGS})
target_compile_options(${PROJECT_NAME} PRIVATE $<$<CONFIG:DEBUG>:${DEBUG_FLAGS}>)

# Set directory containing public headers
target_include_directories(${PROJECT_NAME}
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
)

# Link directories
target_link_directories(${PROJECT_NAME}
    PUBLIC
        ${PCL_LIBRARY_DIRS}
)

# Specify compile definitions
target_compile_definitions(${PROJECT_NAME}
    PUBLIC
        ${PCL_DEFINITIONS}
)

# Link dependent libraries
target_link_libraries(${PROJECT_NAME}
    PUBLIC
        Eigen3::Eigen
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
)

# Set CMake configuration
set(CMAKE_INSTALL_PREFIX "/usr/local" CACHE PATH "Default install prefix")
set(INSTALL_BIN_DIR "${CMAKE_INSTALL_PREFIX}/bin" CACHE PATH "Installation directory for executables")
set(INSTALL_LIB_DIR "${CMAKE_INSTALL_PREFIX}/lib" CACHE PATH "Installation directory for libraries")
set(INSTALL_INCLUDE_DIR "${CMAKE_INSTALL_PREFIX}/include" CACHE PATH "Installation directory for header files")
set(INSTALL_CMAKE_DIR "${CMAKE_INSTALL_PREFIX}/lib/cmake/${PROJECT_NAME}" CACHE PATH "Installation directory for CMake files")

# Export project installation targets
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    RUNTIME DESTINATION "${INSTALL_BIN_DIR}"
    LIBRARY DESTINATION "${INSTALL_LIB_DIR}"
    ARCHIVE DESTINATION "${INSTALL_LIB_DIR}"
    INCLUDES DESTINATION "${INSTALL_INCLUDE_DIR}"
)

# Install the headers
install(DIRECTORY include/ DESTINATION "${INSTALL_INCLUDE_DIR}")

# Create a ConfigVersion.cmake file
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    ${PROJECT_NAME}ConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
    cmake/${PROJECT_NAME}Config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    INSTALL_DESTINATION "${INSTALL_CMAKE_DIR}"
)

# Install the Config and ConfigVersion files
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    DESTINATION "${INSTALL_CMAKE_DIR}"
)

# Install the export targets
install(EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    DESTINATION "${INSTALL_CMAKE_DIR}"
)

# Build tests
if (BUILD_TESTING)
    # Find Google Test
    find_package(GTest REQUIRED)

    # Enable testing
    enable_testing()

    # Discover all test source files
    file(GLOB_RECURSE TEST_SOURCES tests/*.cpp)

    # Set test name
    set(TEST_NAME test_${PROJECT_NAME})

    # Create test executable
    add_executable(${TEST_NAME} ${TEST_SOURCES})

    # Set the compile options for the test executable to always use debug flags
    target_compile_options(${TEST_NAME} PRIVATE ${COMMON_FLAGS})
    target_compile_options(${TEST_NAME} PRIVATE $<$<CONFIG:DEBUG>:${DEBUG_FLAGS}>)

    # Link test executable with Google Test and the project library
    target_link_libraries(${TEST_NAME}
        GTest::GTest
        GTest::Main
        ${PROJECT_NAME}
    )

    # Add test definitions
    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
endif()
