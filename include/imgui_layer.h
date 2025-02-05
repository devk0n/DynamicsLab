#ifndef DYNAMICSLAB_IMGUI_LAYER_H
#define DYNAMICSLAB_IMGUI_LAYER_H

#include <vector>
#include "GLFW/glfw3.h"
#include "glm/glm.hpp"
#include "renderer.h"
#include "dynamics.h"

class ImGuiLayer {
public:
    ImGuiLayer(GLFWwindow* window, Renderer* renderer, Dynamics* dynamics);
    ~ImGuiLayer();

    void renderUI();

    void updateCameraData(const glm::dvec3 &position, const glm::dvec3 &orientation, const double &speed);

private:
    GLFWwindow* m_Window;
    Renderer* m_Renderer;
    Dynamics* m_Dynamics;

    glm::dvec3 m_CameraPosition{};
    glm::dvec3 m_CameraOrientation{};
    double m_CameraSpeed{};

    static void showMainMenu();
    void showSimulationControls();
    static void showRenderingOptions(Renderer*);
    void showDebugWindow() const;

    void showDynamicsData();

};



#endif //DYNAMICSLAB_IMGUI_LAYER_H
