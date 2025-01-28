#ifndef DYNAMICSLAB_IMGUI_LAYER_H
#define DYNAMICSLAB_IMGUI_LAYER_H

#include <vector>
#include "GLFW/glfw3.h"
#include "glm/glm.hpp"

class ImGuiLayer {
public:
    ImGuiLayer(GLFWwindow* window);
    ~ImGuiLayer();

    void renderUI();
    void updateSimulationData(float position, float velocity);

private:
    GLFWwindow* m_Window;

    // Circular buffer for storing simulation data
    static constexpr int historySize = 100;
    std::vector<float> m_PositionHistory = std::vector<float>(historySize, 0.0f);
    std::vector<float> m_VelocityHistory = std::vector<float>(historySize, 0.0f);
    int m_CurrentIndex = 0;

    void showMainMenu();
    void showSimulationControls();
    void showRenderingOptions();
    void showDebugWindow();
    void showSimulationData();

};



#endif //DYNAMICSLAB_IMGUI_LAYER_H
