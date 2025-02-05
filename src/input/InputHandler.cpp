#include "InputHandler.h"
#include <glm/glm.hpp>

// Initialize static members
float InputHandler::s_lastX = 0.0f;
float InputHandler::s_lastY = 0.0f;
bool InputHandler::s_firstMouse = true;

InputHandler::InputHandler(GLFWwindow* window, Camera& camera)
    : m_window(window), m_camera(camera) {

    // Optionally hide and capture cursor for free-look
    // glfwSetInputMode(m_window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // Register a cursor callback if you want to rotate camera with mouse
    glfwSetCursorPosCallback(window, InputHandler::cursorPositionCallback);
}

void InputHandler::processInput(float deltaTime) {
    // Basic WASD movement
    if (glfwGetKey(m_window, GLFW_KEY_W) == GLFW_PRESS) {
        m_camera.moveForward(5.0f * deltaTime);
    }
    if (glfwGetKey(m_window, GLFW_KEY_S) == GLFW_PRESS) {
        m_camera.moveForward(-5.0f * deltaTime);
    }
    if (glfwGetKey(m_window, GLFW_KEY_A) == GLFW_PRESS) {
        m_camera.moveRight(-5.0f * deltaTime);
    }
    if (glfwGetKey(m_window, GLFW_KEY_D) == GLFW_PRESS) {
        m_camera.moveRight(5.0f * deltaTime);
    }
    if (glfwGetKey(m_window, GLFW_KEY_SPACE) == GLFW_PRESS) {
        m_camera.moveUp(5.0f * deltaTime);
    }
    if (glfwGetKey(m_window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS) {
        m_camera.moveUp(-5.0f * deltaTime);
    }

    // You could add other input checks for jumping, sprinting, etc.
}

void InputHandler::cursorPositionCallback(GLFWwindow* window, double xpos, double ypos) {
    if (s_firstMouse) {
        s_lastX = static_cast<float>(xpos);
        s_lastY = static_cast<float>(ypos);
        s_firstMouse = false;
    }

    float xoffset = static_cast<float>(xpos) - s_lastX;
    float yoffset = s_lastY - static_cast<float>(ypos); // Reversed to match typical Y-axis
    s_lastX = static_cast<float>(xpos);
    s_lastY = static_cast<float>(ypos);

    // Adjust sensitivity to taste
    float sensitivity = 0.1f;
    xoffset *= sensitivity;
    yoffset *= sensitivity;

    // Retrieve associated camera from window user pointer if needed:
    // If you store Camera* in glfwSetWindowUserPointer, you can retrieve it here.

    // For demonstration only, you might pass the InputHandler pointer or store the camera differently:
    // InputHandler* handler = static_cast<InputHandler*>(glfwGetWindowUserPointer(window));
    // if (handler) {
    //     handler->m_camera.rotateYaw(xoffset);
    //     handler->m_camera.rotatePitch(yoffset);
    // }

    // In this sample, a static or global approach for Camera usage could be used,
    // but for best design, store the camera pointer in the window user pointer or
    // call back to your existing InputHandler.
}
