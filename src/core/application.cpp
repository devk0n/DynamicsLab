#include "GLFW/glfw3.h"
#include <stdexcept>
#include <iostream>

#include "application.h"
#include "renderer.h"
#include "imgui_layer.h"


Application::Application(int width, int height, const char* title)
    : m_Window(nullptr, glfwDestroyWindow) {

    if (!glfwInit()) {
        throw std::runtime_error("Failed to initialize GLFW");
    }

    glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
    m_Window.reset(glfwCreateWindow(width, height, title, nullptr, nullptr));

    if (!m_Window) {
        glfwTerminate();
        throw std::runtime_error("Failed to create GLFW window");
    }

    glfwMakeContextCurrent(m_Window.get());

    m_Renderer = std::make_unique<Renderer>(m_Window.get());
    glfwSetWindowUserPointer(m_Window.get(), m_Renderer.get());

    // Register callbacks
    glfwSetCursorPosCallback(m_Window.get(), [](GLFWwindow* window, double xpos, double ypos) {
        auto renderer = static_cast<Renderer*>(glfwGetWindowUserPointer(window));
        if (renderer) {
            renderer->handleMouseMovement(xpos, ypos);
        }
    });

    glfwSetMouseButtonCallback(m_Window.get(), [](GLFWwindow* window, int button, int action, int mods) {
        auto renderer = static_cast<Renderer*>(glfwGetWindowUserPointer(window));
        if (renderer) {
            renderer->handleMouseButton(button, action);
        }
    });

    glfwSetCursorPosCallback(m_Window.get(), [](GLFWwindow* window, double xpos, double ypos) {
        auto renderer = static_cast<Renderer*>(glfwGetWindowUserPointer(window));
        if (renderer) {
            renderer->handleMouseMovement(xpos, ypos);
        }
    });

    m_ImGuiLayer = std::make_unique<ImGuiLayer>(m_Window.get());
}


Application::~Application() {
    try {
        m_ImGuiLayer.reset();
        m_Renderer.reset();

        if (m_Window) {
            m_Window.reset();
        }

        glfwTerminate();
    } catch (const std::exception& e) {
        std::cerr << "Exception during Application destruction: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "Unknown error during Application destruction" << std::endl;
    }
}


void Application::run() {
    while (!glfwWindowShouldClose(m_Window.get())) {
        processInput();
        update(0.00025);
        render();

        glfwSwapBuffers(m_Window.get());
        glfwPollEvents();
    }
}


void Application::processInput() {
    static double lastFrameTime = 0.0;
    double currentFrameTime = glfwGetTime();
    double deltaTime = currentFrameTime - lastFrameTime;
    lastFrameTime = currentFrameTime;

    if (m_Window && glfwGetKey(m_Window.get(), GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(m_Window.get(), true);
    }

    if (m_Renderer) {
        m_Renderer->handleKeyboardInput(m_Window.get(), deltaTime);
    }
}


void Application::update(double stepTime) {
    static double position = 0.0;
    static double velocity = 0.0;
    static const double acceleration = -9.8;

    velocity += acceleration * stepTime;
    position += velocity * stepTime;

    if (m_ImGuiLayer) {
        m_ImGuiLayer->updateSimulationData(position, velocity);
    }
}


void Application::render() {
    Renderer::clearScreen({0.1, 0.1, 0.1, 1.0});
    m_Renderer->draw();
    m_ImGuiLayer->renderUI();
}

