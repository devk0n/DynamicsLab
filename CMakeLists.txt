cmake_minimum_required(VERSION 3.30)
project(DynamicsLab)

# Set CMake policies to NEW to eliminate warnings
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS ON CACHE BOOL "" FORCE)
option(CMAKE_WARN_DEPRECATED OFF)
cmake_policy(SET CMP0077 NEW)  # option() honors normal variables
cmake_policy(SET CMP0153 NEW)  # exec_program is deprecated
cmake_policy(SET CMP0167 NEW)  # FindBoost module is removed
cmake_policy(SET CMP0146 NEW)  # FindCUDA module is removed

# —————————————————————————————————————————————————————————————

# Set standards
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# —————————————————————————————————————————————————————————————

include(FetchContent)

# ccache
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
  set(CMAKE_C_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
  set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
endif()

# —————————————————————————————————————————————————————————————

# Eigen
FetchContent_Declare(
    Eigen
    GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
    GIT_TAG 3.4.0)

# Eigen configuration options to eliminate warnings
set(EIGEN_TEST_CXX11 OFF CACHE BOOL "Disable C++11 tests in Eigen" FORCE)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "Don't build Eigen docs" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "Disable Eigen tests" FORCE)
set(EIGEN_TEST_NOQT ON CACHE BOOL "Disable Qt tests in Eigen" FORCE)
set(EIGEN_LEAVE_TEST_IN_ALL_TARGET OFF CACHE BOOL "Don't include tests in all target" FORCE)

FetchContent_MakeAvailable(Eigen)

# —————————————————————————————————————————————————————————————

# GLFW
FetchContent_Declare(glfw GIT_REPOSITORY https://github.com/glfw/glfw.git GIT_TAG 3.4)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(glfw)

# —————————————————————————————————————————————————————————————

# Fetch ImGui
FetchContent_Declare(
        imgui
        GIT_REPOSITORY https://github.com/ocornut/imgui.git
        GIT_TAG v1.91.7
)
FetchContent_MakeAvailable(imgui)
add_library(imgui STATIC
        ${imgui_SOURCE_DIR}/imgui.cpp
        ${imgui_SOURCE_DIR}/imgui_demo.cpp
        ${imgui_SOURCE_DIR}/imgui_draw.cpp
        ${imgui_SOURCE_DIR}/imgui_tables.cpp
        ${imgui_SOURCE_DIR}/imgui_widgets.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui PUBLIC
        ${imgui_SOURCE_DIR}
        ${imgui_SOURCE_DIR}/backends
)
target_link_libraries(imgui glfw OpenGL::GL glad)

# —————————————————————————————————————————————————————————————

# Fetch ImGuizmo
FetchContent_Declare(
    imguizmo
    GIT_REPOSITORY https://github.com/CedricGuillemet/ImGuizmo.git
    GIT_TAG master
    GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(imguizmo)

add_library(imguizmo STATIC
    ${imguizmo_SOURCE_DIR}/ImGuizmo.cpp
)
target_include_directories(imguizmo PUBLIC ${imguizmo_SOURCE_DIR})

# Add these two lines to connect ImGuizmo with ImGui
target_include_directories(imguizmo PUBLIC
    ${imguizmo_SOURCE_DIR}
    ${imgui_SOURCE_DIR}  # Add ImGui headers
)

target_link_libraries(imguizmo PRIVATE imgui)  # Link to ImGui library

# —————————————————————————————————————————————————————————————

# GLM
FetchContent_Declare(glm GIT_REPOSITORY https://github.com/g-truc/glm.git GIT_TAG 1.0.1)
FetchContent_MakeAvailable(glm)

# —————————————————————————————————————————————————————————————

# Build GLAD from source
add_library(glad STATIC
    ${CMAKE_SOURCE_DIR}/external/src/glad.c
)

target_include_directories(glad PUBLIC
    ${CMAKE_SOURCE_DIR}/external/include
)

# —————————————————————————————————————————————————————————————

find_package(OpenGL REQUIRED)

# Sources
set(SOURCES
    source/main.cpp

    source/core/Application.cpp
    source/core/WindowManager.cpp
    source/core/InputManager.cpp
    source/core/Renderer.cpp
    source/core/ShaderManager.cpp
    source/core/Camera.cpp
    source/core/SceneManager.cpp
    source/core/ImGuiManager.cpp

    source/scenes/Primary.cpp

    source/physics/Dynamics.cpp

    source/physics/constraints/DistanceConstraint.cpp
    source/physics/constraints/BallJoint.cpp
    source/physics/constraints/SphericalJoint.cpp
    source/physics/constraints/RevoluteJoint.cpp
    source/physics/constraints/UniversalJoint.cpp

)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/core
    ${CMAKE_CURRENT_SOURCE_DIR}/include/scenes
    ${CMAKE_CURRENT_SOURCE_DIR}/include/physics
    ${CMAKE_CURRENT_SOURCE_DIR}/include/physics/constraints
    ${CMAKE_CURRENT_SOURCE_DIR}/include/physics/objects
    ${CMAKE_CURRENT_SOURCE_DIR}/include/physics/forces
    ${CMAKE_CURRENT_SOURCE_DIR}/include/utilities
)

add_executable(${PROJECT_NAME} ${SOURCES})
target_precompile_headers(${PROJECT_NAME} PRIVATE include/utilities/PCH.h)

target_link_libraries(${PROJECT_NAME} PRIVATE
    glad
    glfw
    glm::glm
    OpenGL::GL
    Eigen3::Eigen
    imgui
    imguizmo
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${imguizmo_SOURCE_DIR}
)

set_target_properties(${PROJECT_NAME} PROPERTIES UNITY_BUILD ON)
