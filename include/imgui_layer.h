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
    void updateSimulationData(double position, double velocity);

private:
    GLFWwindow* m_Window;

    // Circular buffer for storing simulation data
    static constexpr int historySize = 100;
    std::vector<double> m_PositionHistory = std::vector<double>(historySize, 0.0);
    std::vector<double> m_VelocityHistory = std::vector<double>(historySize, 0.0);
    int m_CurrentIndex = 0;

    static void showMainMenu();
    static void showSimulationControls();
    static void showRenderingOptions();
    static void showDebugWindow();
    void showSimulationData();

};



#endif //DYNAMICSLAB_IMGUI_LAYER_H
