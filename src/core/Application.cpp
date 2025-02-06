#include "Application.h"


Application::Application() :
    m_window(nullptr, glfwDestroyWindow),
    m_glfwInitialized(false),
    m_running(false),
    m_lastFrameTime(0) {}

Application::~Application() { shutdown(); }

void Application::run() { mainLoop(); }

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

void Application::mainLoop() {

    while (m_running && !glfwWindowShouldClose(m_window.get())) {
        double currentFrameTime = glfwGetTime();
        double deltaTime = currentFrameTime - m_lastFrameTime;
        m_lastFrameTime = currentFrameTime;

        glfwPollEvents();

        update(deltaTime);

        m_renderer.clear();
        m_renderer.render();

        m_imGuiManager.renderGui();

        glfwSwapBuffers(m_window.get());

    }
}

void Application::update(double deltaTime) {
    InputManager::update(static_cast<float>(deltaTime), m_camera);
    m_renderer.setViewMatrix(m_camera.getViewMatrix());
}

void Application::shutdown() {
    std::cout << "[DynamicsLab] Shutting down... \n";

    // Delete OpenGL objects (Renderer)
    m_renderer.shutdown();

    // Destroy the window and terminate GLFW
    m_window.reset();

    if (m_glfwInitialized) {
        glfwTerminate();
        m_glfwInitialized = false;
    }

    std::cout << "[DynamicsLab] Shutdown complete. \n";
}

bool Application::initializeGLFW() {
    if (!glfwInit()) {
        std::cerr << "[DynamicsLab] Failed to initialize GLFW! \n";
        return false;
    }
    m_glfwInitialized = true;
    std::cout << "[DynamicsLab] GLFW initialized successfully. \n";

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
        std::cerr << "[DynamicsLab] Failed to create GLFW window! \n";
        return false;
    }
    glfwMakeContextCurrent(m_window.get());
    std::cout << "[DynamicsLab] Window created successfully. \n";

    return true;
}

bool Application::initializeGlad() {
    // Load all OpenGL function pointers
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "[DynamicsLab] Failed to initialize Glad \n";
        return false;
    }
    std::cout << "[DynamicsLab] Glad initialized successfully. \n";
    return true;
}

bool Application::initializeImGui() {
    if (!m_imGuiManager.initialize(m_window.get())) {
        std::cerr << "[DynamicsLab] Failed to initialize ImGui \n";
        return false;
    }
    std::cout << "[DynamicsLab] ImGui initialized successfully. \n";
    return true;
}

bool Application::initializeRenderer() {
    if (!m_renderer.initialize()) {
        std::cerr << "[DynamicsLab] Failed to initialize Renderer\n";
        return false;
    }
    std::cout << "[DynamicsLab] Renderer initialized successfully. \n";
    return true;
}
