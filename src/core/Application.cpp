#include "Application.h"


Application::Application() :
    m_window(nullptr, glfwDestroyWindow),
    m_glfwInitialized(false),
    m_running(false),
    m_lastFrameTime(0.0) {}

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

    m_renderer.addCube(std::make_unique<Cube>(glm::vec3(0, 0, 0), glm::vec3(1, 1, 1), glm::vec3(0.9529f, 0.2941f, 0.4902f)));
    m_renderer.addCube(std::make_unique<Cube>(glm::vec3(2, 0, 0), glm::vec3(1, 1, 1), glm::vec3(0.9529f, 0.2941f, 0.4902f)));
    m_renderer.addCube(std::make_unique<Cube>(glm::vec3(4, 0, 0), glm::vec3(1, 1, 1), glm::vec3(0.9529f, 0.2941f, 0.4902f)));
    m_renderer.addCube(std::make_unique<Cube>(glm::vec3(6, 0, 0), glm::vec3(1, 1, 1), glm::vec3(0.9529f, 0.2941f, 0.4902f)));
    m_renderer.addCube(std::make_unique<Cube>(glm::vec3(8, 0, 0), glm::vec3(1, 1, 1), glm::vec3(0.9529f, 0.2941f, 0.4902f)));
    m_renderer.addCube(std::make_unique<Cube>(glm::vec3(10, 0, 0), glm::vec3(1, 1, 1), glm::vec3(0.9529f, 0.2941f, 0.4902f)));
    m_renderer.addCube(std::make_unique<Cube>(glm::vec3(12, 0, 0), glm::vec3(1, 1, 1), glm::vec3(0.9529f, 0.2941f, 0.4902f)));


    while (m_running && !glfwWindowShouldClose(m_window.get())) {
        double currentFrameTime = glfwGetTime();
        double deltaTime = currentFrameTime - m_lastFrameTime;
        m_lastFrameTime = currentFrameTime;

        glfwPollEvents();

        update(deltaTime);

        m_renderer.clear();
        m_renderer.render();

        m_imGuiManager.beginFrame();
        m_imGuiManager.performanceWindow();
        m_imGuiManager.endFrame();

        glfwSwapBuffers(m_window.get());

    }
}

void Application::update(double deltaTime) {
    InputManager::update(static_cast<float>(deltaTime), m_camera);
    m_renderer.setViewMatrix(m_camera.getViewMatrix());
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

