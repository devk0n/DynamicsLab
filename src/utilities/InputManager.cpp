#include "InputManager.h"

GLFWwindow* InputManager::s_window = nullptr;
KeyBindings InputManager::s_keyBindings;  // Default keybindings are initialized automatically

void InputManager::initialize(GLFWwindow* window) {
    s_window = window;
}

void InputManager::update() {
    // No need to loop through keys, just use the struct directly
}

bool InputManager::isKeyPressed(int key) {
    return glfwGetKey(s_window, key) == GLFW_PRESS;
}

bool InputManager::isMouseButtonPressed(int button) {
    return glfwGetMouseButton(s_window, button) == GLFW_PRESS;
}

void InputManager::getMousePosition(double& x, double& y) {
    glfwGetCursorPos(s_window, &x, &y);
}

void InputManager::setKeyBindings(const KeyBindings& bindings) {
    s_keyBindings = bindings;
}

const KeyBindings& InputManager::getKeyBindings() {
    return s_keyBindings;
}
