#include "Application.h"

Application::Application() {}

bool Application::initialize() {
    // 1) Initialize window, GLFW, GLAD
    if (!m_windowManager.initialize()) return false;

    // 2) Initialize Renderer (requires a valid OpenGL context)
    m_renderer.initialize();

    // 3) Ensure OpenGL context is active before ImGui initialization
    if (!m_windowManager.getWindow()) return false;

    // 4) Initialize ImGui
    if (!m_imGuiManager.initialize(m_windowManager.getWindow())) return false;

    // 5) Initialize InputManager
    if (!InputManager::initialize(m_windowManager.getWindow())) return false;

    // 6) Set initial time for deltaTime calculations
    m_lastFrameTime = static_cast<float>(glfwGetTime());




    return true;
}

void Application::update(float deltaTime) {
    InputManager::update(deltaTime, m_camera);
    // m_renderer.setViewMatrix(m_camera.getViewMatrix());
    // m_physicsManager.step(deltaTime);
}

void Application::run() {
    while (!m_windowManager.shouldClose()) {
        auto currentFrameTime = static_cast<float>(glfwGetTime());
        float deltaTime = currentFrameTime - m_lastFrameTime;
        m_lastFrameTime = currentFrameTime;

        m_windowManager.pollEvents();
        update(deltaTime);

        m_renderer.beginFrame();
        // m_renderer.render(m_physicsManager.getRigidBodies(), m_camera.getViewMatrix(), m_camera.getProjectionMatrix(m_windowManager.getAspectRatio()));
        m_imGuiManager.renderGui(m_windowManager.getWindow(), m_renderer, m_camera, m_physicsEngine);


        m_renderer.endFrame();
        m_windowManager.swapBuffers();
    }
}

