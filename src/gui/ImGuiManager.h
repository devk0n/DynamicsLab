#ifndef DYNAMICSLAB_IMGUIMANAGER_H
#define DYNAMICSLAB_IMGUIMANAGER_H

#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"

#include <iostream>

#include "physics/Dynamics.h"

class ImGuiManager {
public:
    ImGuiManager(Dynamics* dynamics);
    ~ImGuiManager() { shutdown(); }

    bool initialize(GLFWwindow* window);

    void renderGui();

    void shutdown();

private:
    Dynamics* m_dynamics = nullptr;

    void beginFrame();
    void dynamicsWindow();
    void performanceWindow() const;
    void endFrame();

    ImGuiWindowFlags m_windowFlags{
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoCollapse
    };

    void controlWindow();
};


#endif
