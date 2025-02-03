#include "Application.h"

Application::Application(int width, int height, const char* title) {
    if (!glfwInit()) {
        throw std::runtime_error("Failed to initialize GLFW!");
    }

    // Now the Window class is fully declared and can be used
    m_window = std::make_unique<Window>(width, height, title);
}

Application::~Application() {
    glfwTerminate();
}

void Application::run() {
    while (!m_window->shouldClose()) {
        m_window->pollEvents();
        m_window->swapBuffers();
    }
}
