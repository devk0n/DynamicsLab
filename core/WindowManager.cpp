#include "WindowManager.h"
#include <iostream>

WindowManager::WindowManager() : m_window(nullptr), m_width(0), m_height(0) {}

WindowManager::~WindowManager() {
    shutdown();
}

bool WindowManager::initialize(int width, int height, const std::string& title) {
    // 1) Initialize GLFW
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW\n";
        return false;
    }

    // (Optional) Setup window hints for desired OpenGL version/profile
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    // You could also add: glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // for Mac

    // 2) Create a window
    m_window = glfwCreateWindow(width, height, title.c_str(), nullptr, nullptr);
    if (!m_window) {
        std::cerr << "Failed to create GLFW window\n";
        glfwTerminate();
        return false;
    }

    // 3) Make the window context current
    glfwMakeContextCurrent(m_window);

    // 4) Load GLAD *after* making the context current
    if (!gladLoadGLLoader((GLADloadproc) glfwGetProcAddress)) {
        std::cerr << "Failed to initialize GLAD\n";
        glfwDestroyWindow(m_window);
        glfwTerminate();
        return false;
    }

    // Optional: Set swap interval (V-sync on)
    glfwSwapInterval(1);

    // 5) Set callback for when the window is resized, so the OpenGL viewport is updated
    glfwSetFramebufferSizeCallback(m_window, [](GLFWwindow* /*window*/, int newWidth, int newHeight) {
        glViewport(0, 0, newWidth, newHeight);
    });

    m_width = width;
    m_height = height;

    // Debug: Check OpenGL info
    std::cout << "OpenGL Version: " << glGetString(GL_VERSION) << std::endl;
    std::cout << "GLSL Version:   " << glGetString(GL_SHADING_LANGUAGE_VERSION) << std::endl;
    std::cout << "Vendor:         " << glGetString(GL_VENDOR) << std::endl;
    std::cout << "Renderer:       " << glGetString(GL_RENDERER) << std::endl;

    return true;
}

void WindowManager::pollEvents() {
    glfwPollEvents();
}

void WindowManager::swapBuffers() {
    glfwSwapBuffers(m_window);
}

bool WindowManager::shouldClose() const {
    return glfwWindowShouldClose(m_window);
}

GLFWwindow* WindowManager::getWindow() const {
    return m_window;
}

float WindowManager::getAspectRatio() const {
    // Avoid potential division by zero if height == 0
    return (m_height == 0) ? 0.0f : static_cast<float>(m_width) / static_cast<float>(m_height);
}

void WindowManager::shutdown() {
    if (m_window) {
        glfwDestroyWindow(m_window);
        m_window = nullptr;
    }
    glfwTerminate();
}
