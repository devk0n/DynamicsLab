//
// Created by devkon on 27/01/2025.
//

#ifndef DYNAMICSLAB_IMGUI_LAYER_H
#define DYNAMICSLAB_IMGUI_LAYER_H

#include "GLFW/glfw3.h"

class ImGuiLayer {
public:
    ImGuiLayer(GLFWwindow* window);
    ~ImGuiLayer();

    void renderUI();

private:
    GLFWwindow* m_Window;

    void showMainMenu();
    void showSimulationControls();
    void showRenderingOptions();
    void showDebugWindow();
};



#endif //DYNAMICSLAB_IMGUI_LAYER_H
