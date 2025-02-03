#include "InputHandler.h"

using namespace glm;

bool InputHandler::m_firstMouse = true;
double InputHandler::m_lastX = 1920 / 2.0;
double InputHandler::m_lastY = 1080 / 2.0;

void InputHandler::processKeyboard(GLFWwindow* window, Camera& camera, double deltaTime) {
    dvec3 direction(0.0);
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        direction += camera.front;

    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        direction -= camera.front;

    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        direction -= normalize(cross(camera.front, camera.up));

    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        direction += normalize(cross(camera.front, camera.up));

    if (length(direction) > 0)
        camera.move(direction, deltaTime);
}

void InputHandler::mouseCallback(GLFWwindow* window, double xpos, double ypos) {
    if (m_firstMouse) {
        m_lastX = xpos;
        m_lastY = ypos;
        m_firstMouse = false;
    }

    double xOffset = xpos - m_lastX;
    double yOffset = m_lastY - ypos;
    m_lastX = xpos;
    m_lastY = ypos;

    Camera* cam = static_cast<Camera*>(glfwGetWindowUserPointer(window));
    if (cam) cam->rotate(xOffset, yOffset);
}
