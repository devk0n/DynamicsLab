#include "Application.h"

Application::Application(int width, int height, const char* title) {

    m_window = std::make_unique<Window>(width, height, title);
    m_renderer = std::make_unique<Renderer>();
    m_guiManager = std::make_unique<GuiManager>(*m_window);

    // Initialize camera (with your preferred aspect ratio)
    float aspectRatio = 1920.0f / 1080.0f;
    m_camera = std::make_unique<Camera>(glm::vec3(0.f, 0.f, 3.f), 45.0f, aspectRatio);

    // Create the input handler, passing the GLFWwindow pointer and the camera
    m_inputHandler = std::make_unique<InputHandler>(m_window->getWindow(), *m_camera);

}

void Application::run() {
    float deltaTime = 0.01;
    while (!m_window->shouldClose()) {
        Window::pollEvents();
        Renderer::clearScreen();

        m_inputHandler->processInput(deltaTime);

        m_renderer->draw();
        m_guiManager->renderGui();

        m_window->swapBuffers();
    }
}

