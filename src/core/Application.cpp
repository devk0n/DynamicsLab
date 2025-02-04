#include "Application.h"

Application::Application(int width, int height, const char* title) {

    m_window = std::make_unique<Window>(width, height, title);
    m_renderer = std::make_unique<Renderer>();
    // m_camera = std::make_unique<Camera>();
    // m_inputHandler = std::make_unique<InputHandler>();
    // m_uiManager = std::make_unique<UIManager>();
}

void Application::run() {
    while (!m_window->shouldClose()) {
        Window::pollEvents();

        // TODO: Process input

        // TODO: Add rendering logic here
        Renderer::clearScreen();

        m_window->swapBuffers();
    }
}

