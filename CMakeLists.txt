cmake_minimum_required(VERSION 3.3)
project(DynamicsLab)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add include directories for project headers
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Eigen include directory
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
set(GLFW_BUILD_DOCS OFF)
set(GLFW_BUILD_EXAMPLES OFF)
set(GLFW_BUILD_TESTS OFF)
FetchContent_MakeAvailable(glfw)

# GLAD
add_library(glad STATIC src/graphics/glad.c)
target_include_directories(glad PUBLIC include)

# GLEW
set(GLEW_INCLUDE_DIR "C:/Users/devkon/Development/Tools/glew-2.2.0/include")
include_directories(${GLEW_INCLUDE_DIR})

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
target_link_libraries(imgui glfw OpenGL::GL)

# Fetch GLM
FetchContent_Declare(
    glm
    GIT_REPOSITORY https://github.com/g-truc/glm.git
    GIT_TAG bf71a834948186f4097caa076cd2663c69a10e1e
)
FetchContent_MakeAvailable(glm)

# Fetch ImPlot
FetchContent_Declare(
    implot
    GIT_REPOSITORY https://github.com/epezent/implot.git
    GIT_TAG v0.16
)
FetchContent_MakeAvailable(implot)

add_library(implot STATIC
    ${implot_SOURCE_DIR}/implot.cpp
    ${implot_SOURCE_DIR}/implot_items.cpp
)
target_include_directories(implot PUBLIC ${implot_SOURCE_DIR})
target_link_libraries(implot imgui)

# Find OpenGL
find_package(OpenGL REQUIRED)

# Source files
set(SOURCES
    src/core/main.cpp
    src/core/application.cpp
    src/graphics/renderer.cpp
    src/ui/imgui_layer.cpp
)

# Add executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Link libraries to the project
target_link_libraries(${PROJECT_NAME} PRIVATE
    imgui
    implot
    glfw
    glad
    glm::glm
    OpenGL::GL
)

# Include the current source directory in the project's include directories
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
