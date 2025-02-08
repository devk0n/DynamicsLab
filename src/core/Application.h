#ifndef DYNAMICSLAB_APPLICATION_H
#define DYNAMICSLAB_APPLICATION_H

#include <memory>
#include <iostream>
#include <stdexcept>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "gui/ImGuiManager.h"
#include "graphics/Renderer.h"
#include "graphics/Camera.h"
#include "input/InputManager.h"
#include "physics/Dynamics.h"


class Application {
public:
    Application();
    ~Application();

    bool initialize();
    void run();

private:
    // Dynamics
    Dynamics m_dynamics;

    // Window-related
    std::unique_ptr<GLFWwindow, decltype(&glfwDestroyWindow)> m_window;

    // Managers and subsystems
    Camera        m_camera;
    ImGuiManager  m_imGuiManager;
    Renderer      m_renderer;
    [[maybe_unused]] InputManager  m_inputManager;

    // State flags
    bool m_glfwInitialized;
    bool m_running;

    double m_lastFrameTime;

    // The main application loop
    void mainLoop();

    // Cleanup everything when shutting down
    void shutdown();

    // Break down the initialization into smaller steps
    // so that `initialize()` doesn’t get too long and complicated.
    bool initializeGLFW();           // Step 1: Initialize GLFW
    bool createMainWindow();         // Step 2: Create the main window
    static bool initializeGlad();    // Step 3: Load OpenGL functions via GLAD
    bool initializeImGui();          // Step 4: Initialize ImGui
    bool initializeRenderer();       // Step 5: Initialize the renderer

    // Methods to keep mainLoop clean
    void update(double deltaTime);
};


#endif
