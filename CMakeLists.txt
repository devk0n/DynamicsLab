cmake_minimum_required(VERSION 3.3)
project(DynamicsLab)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Eigen include directory (adjust this to your local path)
set(EIGEN3_INCLUDE_DIR "C:/Users/devkon/Development/Tools/eigen-3.4.0")
include_directories(${EIGEN3_INCLUDE_DIR})

# Include FetchContent module
include(FetchContent)

# Fetch GLFW
FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.4
)

# Prevent building GLFW extras
set(GLFW_BUILD_DOCS OFF)
set(GLFW_BUILD_EXAMPLES OFF)
set(GLFW_BUILD_TESTS OFF)

FetchContent_MakeAvailable(glfw)

# Fetch ImGui
FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG v1.91.7
)

find_package(OpenGL REQUIRED)

FetchContent_GetProperties(imgui)
if (NOT imgui_POPULATED)
    FetchContent_MakeAvailable(imgui)

    # Add ImGui source files
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

    target_link_libraries(imgui glfw OpenGL::GL)
endif()

# Fetch ImPlot
FetchContent_Declare(
    implot
    GIT_REPOSITORY https://github.com/epezent/implot.git
    GIT_TAG v0.16
)

FetchContent_GetProperties(implot)
if (NOT implot_POPULATED)
    FetchContent_MakeAvailable(implot)

    add_library(implot STATIC
        ${implot_SOURCE_DIR}/implot.cpp
        ${implot_SOURCE_DIR}/implot_items.cpp
    )

    target_include_directories(implot PUBLIC ${implot_SOURCE_DIR})
    target_link_libraries(implot imgui)
endif()

# Source files
set(SOURCES
        main.cpp
        dynamics/Simulation.cpp
        graphics/graphics_manager.cpp
)

# Add executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Link libraries to the project
target_link_libraries(${PROJECT_NAME} PRIVATE imgui implot glfw OpenGL::GL)
