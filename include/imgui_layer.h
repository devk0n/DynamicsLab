#ifndef DYNAMICSLAB_IMGUI_LAYER_H
#define DYNAMICSLAB_IMGUI_LAYER_H

#include <vector>
#include "GLFW/glfw3.h"
#include "glm/glm.hpp"
#include "renderer.h"

class ImGuiLayer {
public:
    ImGuiLayer(GLFWwindow* window, Renderer* renderer);
    ~ImGuiLayer();

    void renderUI();

    void updateCameraData(const glm::dvec3 &position, const glm::dvec3 &orientation, const double &speed);

private:
    GLFWwindow* m_Window;
    Renderer* m_Renderer;

    glm::dvec3 m_CameraPosition{};
    glm::dvec3 m_CameraOrientation{};
    double m_CameraSpeed{};

    // Circular buffer for storing simulation data
    static constexpr int historySize = 100;
    std::vector<double> m_PositionHistory = std::vector<double>(historySize, 0.0);
    std::vector<double> m_VelocityHistory = std::vector<double>(historySize, 0.0);
    int m_CurrentIndex = 0;

    static void showMainMenu();
    static void showSimulationControls();
    static void showRenderingOptions(Renderer*);
    void showDebugWindow() const;
    void showSimulationData();

};



#endif //DYNAMICSLAB_IMGUI_LAYER_H
