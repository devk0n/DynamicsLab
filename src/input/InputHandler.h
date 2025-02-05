#ifndef DYNAMICSLAB_INPUTHANDLER_H
#define DYNAMICSLAB_INPUTHANDLER_H

#include <GLFW/glfw3.h>
#include "../gui/ImGuiManager.h"

class InputHandler {
public:
    explicit InputHandler(GLFWwindow* window);
    ~InputHandler();

private:
    GLFWwindow* m_window;

};

#endif // DYNAMICSLAB_INPUTHANDLER_H