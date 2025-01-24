cmake_minimum_required(VERSION 3.5)
project(DynamicsLab)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include FetchContent module
include(FetchContent)

# Fetch GLFW
FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.4
)

# Prevent building GLFW extras like tests and examples
set(GLFW_BUILD_DOCS OFF)
set(GLFW_BUILD_EXAMPLES OFF)
set(GLFW_BUILD_TESTS OFF)

# Make GLFW available
FetchContent_MakeAvailable(glfw)

# Source files
set(SOURCES
    main.cpp
)

# Add executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Link GLFW to your project
target_link_libraries(${PROJECT_NAME} PRIVATE glfw)
