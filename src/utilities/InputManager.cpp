#include "InputManager.h"

GLFWwindow* InputManager::s_window = nullptr;
KeyBindings InputManager::s_keyBindings; // Automatically initialized

void InputManager::initialize(GLFWwindow* window) {
    s_window = window;
}

void InputManager::update(float deltaTime, Camera& camera) {
    camera.processKeyboard(s_keyBindings, deltaTime); // Pass the struct instead of individual keys

    // Mouse handling
    bool rightMouseHeld = isMouseButtonPressed(GLFW_MOUSE_BUTTON_RIGHT);
    double mouseX, mouseY;
    getMousePosition(mouseX, mouseY);
    camera.processMouseMovement(static_cast<float>(mouseX), static_cast<float>(mouseY), rightMouseHeld);

    // Toggle cursor visibility
    glfwSetInputMode(s_window, GLFW_CURSOR, rightMouseHeld ? GLFW_CURSOR_DISABLED : GLFW_CURSOR_NORMAL);
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
