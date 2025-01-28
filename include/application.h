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

    glm::vec3 cameraPos = glm::vec3(0.0f, 0.0f, 3.0f);
    glm::vec3 cameraFront = glm::vec3(0.0f, 0.0f, -1.0f);
    glm::vec3 cameraUp = glm::vec3(0.0f, 1.0f, 0.0f);

    float yaw = -90.0f;  // Initial yaw (look forward)
    float pitch = 0.0f;  // Initial pitch (no tilt)
    bool rightMouseHeld = false; // State for right mouse button

    void processInput();
    void update(float deltaTime);
    void render();

};


#endif //DYNAMICSLAB_APPLICATION_H
