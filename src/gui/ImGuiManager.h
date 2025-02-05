#ifndef DYNAMICSLAB_IMGUIMANAGER_H
#define DYNAMICSLAB_IMGUIMANAGER_H

#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"

class ImGuiManager {
public:
    // Initialize ImGui with the provided GLFW window
    bool initialize(GLFWwindow* window);

    void renderGui();

    // Shutdown ImGui and clean up resources
    void shutdown();
private:
    void beginFrame();
    void endFrame();

    void performanceWindow() const;

    ImGuiWindowFlags m_windowFlags =
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoCollapse;
};


#endif //DYNAMICSLAB_IMGUIMANAGER_H
