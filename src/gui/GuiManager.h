#ifndef DYNAMICSLAB_GUIMANAGER_H
#define DYNAMICSLAB_GUIMANAGER_H

#include "../window/Window.h"

#include <imgui.h>
#include <backends/imgui_impl_glfw.h>
#include <backends/imgui_impl_opengl3.h>

#include <glm/glm.hpp>
#include <GLFW/glfw3.h>

class GuiManager {
public:
    explicit GuiManager(Window& window);
    ~GuiManager();

    void renderGui();

private:
    void beginFrame();
    void endFrame();

    void performanceWindow() const;

    ImGuiWindowFlags m_windowFlags =
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoCollapse;

};


#endif //DYNAMICSLAB_GUIMANAGER_H
