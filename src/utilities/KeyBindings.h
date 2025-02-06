#ifndef DYNAMICSLAB_KEYBINDINGS_H
#define DYNAMICSLAB_KEYBINDINGS_H

#include <GLFW/glfw3.h>

struct KeyBindings {
    int moveForward  = GLFW_KEY_W;
    int moveBackward = GLFW_KEY_S;
    int moveLeft     = GLFW_KEY_A;
    int moveRight    = GLFW_KEY_D;
    int moveUp       = GLFW_KEY_LEFT_SHIFT;
    int moveDown     = GLFW_KEY_LEFT_CONTROL;
};

#endif
