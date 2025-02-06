#include "Application.h"


Application::Application() :
    m_window(nullptr, glfwDestroyWindow),
    m_glfwInitialized(false),
    m_running(false),
    m_lastFrameTime(0.0) {

}

Application::~Application() {
    shutdown();
}

void Application::run() {
    mainLoop();
}

bool Application::initialize() {
    // Combine calls using short-circuit logic
    if (!initializeGLFW()   ||
        !createMainWindow() ||
        !initializeGlad()   ||
        !initializeImGui()  ||
        !initializeRenderer()) {

        shutdown();
        return false;
    }

    InputManager::initialize(m_window.get());

    m_running = true;
    return true;
}

bool Application::initializeGLFW() {
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return false;
    }
    m_glfwInitialized = true;
    std::cout << "GLFW initialized successfully." << std::endl;

    // Configure GLFW window properties
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_DEPTH_BITS, 24);

    return true;
}

bool Application::createMainWindow() {
    // Create window
    m_window.reset(glfwCreateWindow(1920, 1280, "DynamicsLab", nullptr, nullptr));
    if (!m_window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        return false;
    }
    glfwMakeContextCurrent(m_window.get());
    std::cout << "Window created successfully." << std::endl;

    return true;
}

bool Application::initializeGlad() {
    // Load all OpenGL function pointers
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "Failed to initialize Glad" << std::endl;
        return false;
    }
    std::cout << "Glad initialized successfully." << std::endl;
    return true;
}

bool Application::initializeImGui() {
    if (!m_imGuiManager.initialize(m_window.get())) {
        std::cerr << "Failed to initialize ImGui" << std::endl;
        return false;
    }
    std::cout << "ImGui initialized successfully." << std::endl;
    return true;
}

bool Application::initializeRenderer() {
    if (!m_renderer.initialize()) {
        std::cerr << "Failed to initialize Renderer" << std::endl;
        return false;
    }
    std::cout << "Renderer initialized successfully." << std::endl;
    return true;
}


void Application::mainLoop() {
    while (m_running && !glfwWindowShouldClose(m_window.get())) {
        double currentFrameTime = glfwGetTime();
        double deltaTime = currentFrameTime - m_lastFrameTime;
        m_lastFrameTime = currentFrameTime;

        glfwPollEvents(); // Pump window events

        update(deltaTime);   // <--- Camera input, etc.
        renderFrame();       // <--- Clear, ImGui, draw, swap
    }
}

void Application::update(double deltaTime) {
    InputManager::update(static_cast<float>(deltaTime), m_camera);
    m_renderer.setViewMatrix(m_camera.getViewMatrix());
}

void Application::renderFrame() {
    // 1) Clear screen
    m_renderer.clear();

    // 2) ImGui pass
    m_imGuiManager.renderGui();

    // 3) Render 3D scene (grid floor, etc.)
    m_renderer.render();

    // 4) Present
    glfwSwapBuffers(m_window.get());
}

void Application::shutdown() {
    std::cout << "[Application] Shutting down...\n";

    // Delete OpenGL objects (Renderer)
    m_renderer.shutdown();

    // Destroy the window and terminate GLFW
    m_window.reset();

    if (m_glfwInitialized) {
        glfwTerminate();
        m_glfwInitialized = false;
    }

    std::cout << "[Application] Shutdown complete.\n";
}

