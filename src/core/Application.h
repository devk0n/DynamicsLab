#ifndef DYNAMICSLAB_APPLICATION_H
#define DYNAMICSLAB_APPLICATION_H

#include <iostream>
#include <memory>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include "../gui/ImGuiManager.h"
#include "../input/InputHandler.h"
#include "../graphics/Renderer.h"

class Application {
public:

    explicit Application();
    ~Application();

    void run();

private:

    bool initialize();
    void mainLoop();
    void shutdown();

    std::unique_ptr<GLFWwindow, void(*)(GLFWwindow*)> m_window;
    ImGuiManager m_imGuiManager;
    std::unique_ptr<InputHandler> m_inputHandler;
    Renderer m_renderer;

    bool m_glfwInitialized;
    bool m_running;
};

#endif // DYNAMICSLAB_APPLICATION_H
