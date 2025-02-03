#ifndef DYNAMICSLAB_INPUTHANDLER_H
#define DYNAMICSLAB_INPUTHANDLER_H

#include "GLFW/glfw3.h"
#include "../graphics/Camera.h"

class InputHandler {
public:
    static void processKeyboard(GLFWwindow* window, Camera& camera, double deltaTime);
    static void mouseCallback(GLFWwindow* window, double xpos, double ypos);

private:
    static bool m_firstMouse;
    static double m_lastX, m_lastY;

};


#endif //DYNAMICSLAB_INPUTHANDLER_H
