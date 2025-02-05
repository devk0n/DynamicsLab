#include "Application.h"

Application::Application()
    : m_window(nullptr, glfwDestroyWindow) {

    if (!initialize()) {
        throw std::runtime_error("Failed to initialize application");
    }
}

Application::~Application() {
    shutdown();
}

void Application::run() {
    mainLoop();
}

bool Application::initialize() {
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return false;
    }

    m_window.reset(glfwCreateWindow(1920, 1280, "DynamicsLab", nullptr, nullptr));
    if (!m_window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(m_window.get());

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "Failed to initialize Glad" << std::endl;
        return false;
    }

    /*
    if (!imGuiManager.Initialize(m_window)) {
        std::cerr << "Failed to initialize ImGui" << std::endl;
        return false;
    }

    if (!renderer.Initialize()) {
        std::cerr << "Failed to initialize Renderer" << std::endl;
        return false;
    }
    */
    isRunning = true;
    return true;
}

void Application::mainLoop() {
    while (isRunning && !glfwWindowShouldClose(m_window.get())) {
        // inputHandler.PollEvents();
        // renderer.Clear();

        // Update and render the scene

        // imGuiManager.BeginFrame();
        // Add ImGui widgets here
        // imGuiManager.EndFrame();

        glfwSwapBuffers(m_window.get());
        glfwPollEvents();
    }
}

void Application::shutdown() {
     m_window.reset();
    glfwTerminate();
}


