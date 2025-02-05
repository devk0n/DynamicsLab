#ifndef DYNAMICSLAB_APPLICATION_H
#define DYNAMICSLAB_APPLICATION_H

#include <memory>
#include <iostream>
#include <stdexcept>
#include "../gui/ImGuiManager.h"
#include "../graphics/Renderer.h"
#include "../graphics/Camera.h"
#include <GLFW/glfw3.h>

class Application {
public:
    Application();
    ~Application();

    void run();

private:
    std::unique_ptr<GLFWwindow, decltype(&glfwDestroyWindow)> m_window;
    bool m_glfwInitialized;
    bool m_running;

    float m_lastFrameTime;

    Camera m_camera;  // Our WASD + mouse-look camera

    ImGuiManager m_imGuiManager;
    Renderer m_renderer;

    bool initialize();
    void mainLoop();
    void shutdown();

    // --- New methods to keep mainLoop clean ---
    void update(float deltaTime);
    void renderFrame();
};

#endif // DYNAMICSLAB_APPLICATION_H
