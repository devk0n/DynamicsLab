#ifndef DYNAMICSLAB_GRAPHICS_MANAGER_H
#define DYNAMICSLAB_GRAPHICS_MANAGER_H

#include "GLFW/glfw3.h"

class GraphicsManager {
public:
    GraphicsManager();
    ~GraphicsManager();

    void run();

private:
    GLFWwindow* window{};

    int WINDOW_WIDTH{};
    int WINDOW_HEIGHT{};
    const char* WINDOW_TITLE{};

    static void initializeGLFW();
    void createWindow();
    void mainLoop();
    void cleanUp();
};


#endif //DYNAMICSLAB_GRAPHICS_MANAGER_H
