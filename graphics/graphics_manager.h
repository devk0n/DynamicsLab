#ifndef DYNAMICSLAB_GRAPHICS_MANAGER_H
#define DYNAMICSLAB_GRAPHICS_MANAGER_H

#include <memory>

#include "GLFW/glfw3.h"

class GraphicsManager {
public:
    GraphicsManager();
    ~GraphicsManager();

    void run();

private:
    std::unique_ptr<GLFWwindow, void(*)(GLFWwindow*)> window;


    int WINDOW_WIDTH{};
    int WINDOW_HEIGHT{};
    const char* WINDOW_TITLE{};

    static void initializeGLFW();
    void createWindow();
    void mainLoop();
    void cleanUp();

    void renderGUI();

    void showSimulationWindow();

    void showControlWindow();

    void showDebugWindow();

    void initializeImGui();
};


#endif //DYNAMICSLAB_GRAPHICS_MANAGER_H
