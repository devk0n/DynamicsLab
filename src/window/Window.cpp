#include "Window.h"

Window::Window(int width, int height, const char* title)
    : m_window(glfwCreateWindow(width, height, title, nullptr, nullptr), glfwDestroyWindow) {

    if (!m_window) {
        throw std::runtime_error("Failed to create GLFW window!");
    }
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
