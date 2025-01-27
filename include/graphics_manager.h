#ifndef DYNAMICSLAB_GRAPHICS_MANAGER_H
#define DYNAMICSLAB_GRAPHICS_MANAGER_H

#include <memory>

#include "GLFW/glfw3.h"

class GraphicsManager {
public:
    GraphicsManager();
    ~GraphicsManager();

    void run();

    double time{};

private:
    std::unique_ptr<GLFWwindow, void(*)(GLFWwindow*)> window;

    int WINDOW_WIDTH{};
    int WINDOW_HEIGHT{};
    const char* WINDOW_TITLE{};



    static void initializeGLFW();
    void createWindow();
    void mainLoop();
    void cleanUp();

    void initializeImGui();

    void showSimulationWindow();
    // void showControlWindow();
    void showDebugWindow();

    void renderGUI();

    void showControlWindow();
};

#endif //DYNAMICSLAB_GRAPHICS_MANAGER_H
