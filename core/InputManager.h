#ifndef INPUTMANAGER_H
#define INPUTMANAGER_H

#include "Camera.h"
#include <GLFW/glfw3.h>

class InputManager {
public:
    static bool initialize(GLFWwindow* window);
    static void update(float deltaTime, Camera& camera);

private:
    static GLFWwindow* s_window;
};


#endif // INPUTMANAGER_H
