#include "Application.h"

Application::Application(int width, int height, const char* title)
    : m_window(width, height, title), m_camera(glm::vec3(0.0f, 0.0f, 3.0f)) {

    // Pass camera reference to input handler
    glfwSetCursorPosCallback(m_window.getWindow(), InputHandler::mouseCallback);
    glfwSetWindowUserPointer(m_window.getWindow(), &m_camera);
}

void Application::run() {
    float lastFrame = 0.0f;

    while (!m_window.shouldClose()) {
        float currentFrame = glfwGetTime();
        float deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        // Process input
        InputHandler::processKeyboard(m_window.getWindow(), m_camera, deltaTime);

        // Poll events and swap buffers
        m_window.pollEvents();
        m_window.swapBuffers();
    }
}