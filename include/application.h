#ifndef DYNAMICSLAB_APPLICATION_H
#define DYNAMICSLAB_APPLICATION_H

#include <memory>

#include "GLFW/glfw3.h"

#include "renderer.h"
#include "imgui_layer.h"

class Application {
public:
    Application(int width, int height, const char* title);
    ~Application();

    void run();
private:
    std::unique_ptr<GLFWwindow, void(*)(GLFWwindow*)> m_Window;
    std::unique_ptr<Renderer> m_Renderer;
    std::unique_ptr<ImGuiLayer> m_ImGuiLayer;

    void processInput();
    void update();
    void render();

    void captureScreenshot();
};


#endif //DYNAMICSLAB_APPLICATION_H
