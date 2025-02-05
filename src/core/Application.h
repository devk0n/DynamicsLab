#ifndef DYNAMICSLAB_APPLICATION_H
#define DYNAMICSLAB_APPLICATION_H

#include <memory>
#include <iostream>
#include <stdexcept>
#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "../gui/ImGuiManager.h"
#include "../graphics/Renderer.h"
#include "../graphics/Camera.h"


class Application {
public:
    Application();
    ~Application();

    void run();

private:
    // Window-related
    std::unique_ptr<GLFWwindow, decltype(&glfwDestroyWindow)> m_window;

    // Managers and subsystems
    Camera        m_camera;
    ImGuiManager  m_imGuiManager;
    Renderer      m_renderer;

    // State flags
    bool m_glfwInitialized;
    bool m_running;

    float m_lastFrameTime;

    // The main game/application loop
    void mainLoop();

    // Cleanup everything when shutting down
    void shutdown();

    //
    // 1. Orchestrate the entire initialization process
    //
    bool initialize();

    //
    // 2. Break down the initialization into smaller steps
    //    so that `initialize()` doesn’t get too long and complicated.
    //
    bool initializeGLFW();           // Step 1: Initialize GLFW
    bool createMainWindow();         // Step 2: Create the main window
    bool initializeGlad();           // Step 3: Load OpenGL functions via GLAD
    bool initializeImGui();          // Step 4: Initialize ImGui
    bool initializeRenderer();       // Step 5: Initialize the renderer

    //
    // 3. Methods to keep mainLoop clean
    //
    void update(float deltaTime);
    void renderFrame();
};


#endif // DYNAMICSLAB_APPLICATION_H
