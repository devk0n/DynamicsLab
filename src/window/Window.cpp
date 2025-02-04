#include "Window.h"

#include "GLFW/glfw3.h"
#include <stdexcept>

Window::Window(int width, int height, const char* title)
    : m_window(nullptr, glfwDestroyWindow) {

    if (!glfwInit()) {
        throw std::runtime_error("Failed to initialize GLFW!");
    }

    // Set OpenGL version
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // Directly assign GLFW window to m_window
    m_window.reset(glfwCreateWindow(width, height, title, nullptr, nullptr));
    if (!m_window) {
        glfwTerminate();
        throw std::runtime_error("Failed to create GLFW window!");
    }

    glfwMakeContextCurrent(m_window.get());

    // Load OpenGL function pointers using GLAD
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        throw std::runtime_error("Failed to initialize GLAD!");
    }

    std::cout << "OpenGL version: " << glGetString(GL_VERSION) << std::endl;
}


Window::~Window() {
    // Destructor: Cleanup GLFW when the object is destroyed
    m_window.reset(); // Explicitly destroy the window
    glfwTerminate();
}

bool Window::shouldClose() const {
    return glfwWindowShouldClose(m_window.get());
}

void Window::pollEvents() {
    glfwPollEvents();
}

void Window::swapBuffers() {
    glfwSwapBuffers(m_window.get());
}
