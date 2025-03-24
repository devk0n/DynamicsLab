cmake_minimum_required(VERSION 3.30)
project(DynamicsLab)

# Set CMake policies to NEW to eliminate warnings
cmake_policy(SET CMP0077 NEW)  # option() honors normal variables
cmake_policy(SET CMP0153 NEW)  # exec_program is deprecated
cmake_policy(SET CMP0167 NEW)  # FindBoost module is removed
cmake_policy(SET CMP0146 NEW)  # FindCUDA module is removed

if(MSVC)
  add_compile_options(/std:c++20)
else()
  add_compile_options(-std=c++20)
endif()

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Enable ccache
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    message(STATUS "Using ccache: ${CCACHE_PROGRAM}")
    set(CMAKE_C_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
    set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
endif()

# Force Eigen to recognize C++11 support
set(EIGEN_COMPILER_SUPPORT_CPP11 TRUE CACHE BOOL "Force C++11 support" FORCE)
set(EIGEN_TEST_CXX11 ON CACHE BOOL "Enable C++11 support in Eigen" FORCE)

# Disable unwanted Eigen components to reduce warnings
set(EIGEN_BUILD_DOC OFF CACHE BOOL "Don't build Eigen documentation" FORCE)
set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "Don't build pkg-config files" FORCE)
set(EIGEN_DISABLE_CXX11_TESTSUITE OFF CACHE BOOL "Enable CXX11 testsuite" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "Disable Eigen tests" FORCE)

# Fetch Eigen
include(FetchContent)
FetchContent_Declare(
    Eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3.4.0
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(Eigen)

# Source files
set(SOURCES source/main.cpp)

# Executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Link Eigen and Mingw's math library
target_link_libraries(${PROJECT_NAME} PRIVATE Eigen3::Eigen)

# Include directories
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/source
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/core
)