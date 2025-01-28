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
    glfwSetWindowUserPointer(m_Window.get(), m_Renderer.get()); // Link Renderer to the window

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
        update(0.00025f);
        render();

        glfwSwapBuffers(m_Window.get());
        glfwPollEvents();
    }
}

void Application::processInput() {
    // Poll input states
    double xpos, ypos;
    glfwGetCursorPos(m_Window.get(), &xpos, &ypos);

    if (glfwGetMouseButton(m_Window.get(), GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS) {
        m_Renderer->handleMouseMovement(xpos, ypos);
    }
}

void Application::update(float stepTime) {
    static float position = 0.0f;
    static float velocity = 0.0f;
    static const float acceleration = -9.8f;

    velocity += acceleration * stepTime;
    position += velocity * stepTime;

    if (m_ImGuiLayer) {
        m_ImGuiLayer->updateSimulationData(position, velocity);
    }
}

void Application::render() {
    m_Renderer->clearScreen({0.1f, 0.1f, 0.1f, 1.0f});
    m_Renderer->draw();
    m_ImGuiLayer->renderUI();
}

