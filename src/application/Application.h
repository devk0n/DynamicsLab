#ifndef DYNAMICSLAB_APPLICATION_H
#define DYNAMICSLAB_APPLICATION_H

#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include "../main/infrastructure/Window.h"
#include "../main/renderer/Camera.h"
#include "../main/input/InputHandler.h"

class Application {
public:
    Application(int width, int height, const char* title);
    void run();
private:
    Window m_window;
    Camera m_camera;
};


#endif //DYNAMICSLAB_APPLICATION_H
