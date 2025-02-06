#ifndef DYNAMICSLAB_IMGUIMANAGER_H
#define DYNAMICSLAB_IMGUIMANAGER_H

#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"

class ImGuiManager {
public:
    ImGuiManager() = default;
    ~ImGuiManager() { shutdown(); }

    bool initialize(GLFWwindow* window);

    void beginFrame();
    void performanceWindow() const;
    void endFrame();

    void shutdown();

private:



    ImGuiWindowFlags m_windowFlags{
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoCollapse
    };
};


#endif //DYNAMICSLAB_IMGUIMANAGER_H
