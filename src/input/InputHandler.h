#ifndef DYNAMICSLAB_INPUTHANDLER_H
#define DYNAMICSLAB_INPUTHANDLER_H

#include "GLFW/glfw3.h"
#include "../graphics/Camera.h"

class Window; // Forward declaration if you store or need a Window reference

class InputHandler {
public:
    InputHandler(GLFWwindow* window, Camera& camera);

    // Call this every frame to check keyboard/mouse states.
    void processInput(float deltaTime);

    // Mouse callback if desired:
    static void cursorPositionCallback(GLFWwindow* window, double xpos, double ypos);

private:
    GLFWwindow* m_window;
    Camera& m_camera;

    static float s_lastX;
    static float s_lastY;
    static bool s_firstMouse;
};

#endif //DYNAMICSLAB_INPUTHANDLER_H