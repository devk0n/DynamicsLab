cmake_minimum_required(VERSION 3.5)
project(DynamicsLab)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)

# Source files
set(SOURCES
        main.cpp
)

# Add executable
add_executable(${PROJECT_NAME} ${SOURCES})
