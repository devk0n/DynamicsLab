#include "Application.h"

Application::Application()
    : m_window(nullptr, glfwDestroyWindow),
      m_glfwInitialized(false),
      m_running(false) {

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
    m_glfwInitialized = true;
    std::cout << "GLFW initialized successfully." << std::endl;

    m_window.reset(glfwCreateWindow(1920, 1280, "DynamicsLab", nullptr, nullptr));
    if (!m_window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        shutdown();
        return false;
    }
    std::cout << "Window created successfully." << std::endl;

    glfwMakeContextCurrent(m_window.get());

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "Failed to initialize Glad" << std::endl;
        shutdown();
        return false;
    }
    std::cout << "Glad initialized successfully." << std::endl;

    if (!m_imGuiManager.initialize(m_window.get())) {
        std::cerr << "Failed to initialize ImGui" << std::endl;
        shutdown();
        return false;
    }
    std::cout << "ImGui initialized successfully." << std::endl;

    if (!m_renderer.initialize()) {
        std::cerr << "Failed to initialize Renderer" << std::endl;
        shutdown();
        return false;
    }
    std::cout << "Renderer initialized successfully." << std::endl;

    m_inputHandler = std::make_unique<InputHandler>(m_window.get());

    m_running = true;
    return true;
}


void Application::mainLoop() {
    while (m_running && !glfwWindowShouldClose(m_window.get())) {
        glfwPollEvents();

        m_renderer.clear();
        m_imGuiManager.renderGui();
        m_renderer.render();

        glfwSwapBuffers(m_window.get());
    }
}

void Application::shutdown() {
    m_imGuiManager.shutdown();
    m_inputHandler.reset();
    m_window.reset();

    if (m_glfwInitialized) {
        glfwTerminate();
        m_glfwInitialized = false;
    }
}



