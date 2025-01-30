#ifndef DYNAMICSLAB_APPLICATION_H
#define DYNAMICSLAB_APPLICATION_H

#include <memory>

#include "GLFW/glfw3.h"

#include "renderer.h"
#include "imgui_layer.h"
#include "dynamics.h"

class Application {
public:
    Application(int width, int height, const char* title);
    ~Application();

    void run();
private:
    std::unique_ptr<GLFWwindow, void(*)(GLFWwindow*)> m_window;
    std::unique_ptr<Renderer> m_renderer;
    std::unique_ptr<ImGuiLayer> m_imguiLayer;
    std::unique_ptr<Dynamics> m_dynamics;

    void render();
    void update();

    void processInput();
    void captureScreenshot();
};

#endif //DYNAMICSLAB_APPLICATION_H
