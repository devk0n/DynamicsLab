cmake_minimum_required(VERSION 3.0)
project(DynamicsLab)

# Set C++ standard
set(CMAKE_CXX_STANDARD 26)

# Find Vulkan
find_package(Vulkan REQUIRED)

# Source files
set(SOURCES
        main.cpp
)

# Add executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Link Vulkan library
target_link_libraries(${PROJECT_NAME} Vulkan::Vulkan)