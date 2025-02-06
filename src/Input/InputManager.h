#ifndef DYNAMICSLAB_INPUTMANAGER_H
#define DYNAMICSLAB_INPUTMANAGER_H

#include "GLFW/glfw3.h"
#include "KeyBindings.h"
#include "graphics/Camera.h"


class InputManager {
public:
    static void initialize(GLFWwindow* window);
    static void update(float deltaTime, Camera& camera);

    static bool isKeyPressed(int key);
    static bool isMouseButtonPressed(int button);
    static void getMousePosition(double& x, double& y);

    static void setKeyBindings(const KeyBindings& bindings);
    static const KeyBindings& getKeyBindings();

private:
    static GLFWwindow* s_window;
    static KeyBindings s_keyBindings;
};


#endif
