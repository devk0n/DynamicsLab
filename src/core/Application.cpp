#include "Application.h"


Application::Application()
    : m_window(nullptr, glfwDestroyWindow)
    , m_glfwInitialized(false)
    , m_running(false)
    , m_lastFrameTime(0.0f) {
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
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // Before glfwCreateWindow:
    glfwWindowHint(GLFW_DEPTH_BITS, 24); // ensure we have a depth buffer


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

    // Start running
    m_running = true;
    return true;
}

void Application::mainLoop() {
    while (m_running && !glfwWindowShouldClose(m_window.get())) {
        float currentFrameTime = static_cast<float>(glfwGetTime());
        float deltaTime = currentFrameTime - m_lastFrameTime;
        m_lastFrameTime = currentFrameTime;

        glfwPollEvents(); // Pump window events

        update(deltaTime);   // <--- Camera input, etc.
        renderFrame();       // <--- Clear, ImGui, draw, swap
    }
}

void Application::update(float deltaTime) {
    // 1) Keyboard (WASD)
    bool wKey = (glfwGetKey(m_window.get(), GLFW_KEY_W) == GLFW_PRESS);
    bool sKey = (glfwGetKey(m_window.get(), GLFW_KEY_S) == GLFW_PRESS);
    bool aKey = (glfwGetKey(m_window.get(), GLFW_KEY_A) == GLFW_PRESS);
    bool dKey = (glfwGetKey(m_window.get(), GLFW_KEY_D) == GLFW_PRESS);
    m_camera.processKeyboard(wKey, sKey, aKey, dKey, deltaTime);

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
    m_imGuiManager.shutdown();
    m_window.reset();

    if (m_glfwInitialized) {
        glfwTerminate();
        m_glfwInitialized = false;
    }
}
