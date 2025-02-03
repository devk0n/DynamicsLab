#ifndef DYNAMICSLAB_INPUTHANDLER_H
#define DYNAMICSLAB_INPUTHANDLER_H

#include <GLFW/glfw3.h>
#include "../renderer/Camera.h"

class InputHandler {
public:
    static void processKeyboard(GLFWwindow* window, Camera& camera, float deltaTime);
    static void mouseCallback(GLFWwindow* window, double xpos, double ypos);

private:
    static bool firstMouse;
    static float lastX, lastY;

};


#endif //DYNAMICSLAB_INPUTHANDLER_H
