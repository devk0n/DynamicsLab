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
    // 1) Keyboard (WASD)
    bool wKey = (glfwGetKey(m_window.get(), GLFW_KEY_W) == GLFW_PRESS);
    bool sKey = (glfwGetKey(m_window.get(), GLFW_KEY_S) == GLFW_PRESS);
    bool aKey = (glfwGetKey(m_window.get(), GLFW_KEY_A) == GLFW_PRESS);
    bool dKey = (glfwGetKey(m_window.get(), GLFW_KEY_D) == GLFW_PRESS);
    m_camera.processKeyboard(wKey, sKey, aKey, dKey, static_cast<float>(deltaTime));

    // 2) Mouse look if right button is held
    bool rightMouseHeld = (glfwGetMouseButton(m_window.get(), GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
    double mouseX, mouseY;
    glfwGetCursorPos(m_window.get(), &mouseX, &mouseY);
    m_camera.processMouseMovement(static_cast<float>(mouseX),
                                  static_cast<float>(mouseY),
                                  rightMouseHeld);

    // Optionally hide the cursor while right mouse is held
    if (rightMouseHeld) {
        glfwSetInputMode(m_window.get(), GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    } else {
        glfwSetInputMode(m_window.get(), GLFW_CURSOR, GLFW_CURSOR_NORMAL);
    }

    // 3) Update renderer's view matrix from camera
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

