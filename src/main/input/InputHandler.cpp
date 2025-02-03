#include "InputHandler.h"

bool InputHandler::firstMouse = true;
float InputHandler::lastX = 800 / 2.0f;
float InputHandler::lastY = 600 / 2.0f;

void InputHandler::processKeyboard(GLFWwindow* window, Camera& camera, float deltaTime) {
    glm::vec3 direction(0.0f);
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        direction += camera.front;
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        direction -= camera.front;
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        direction -= glm::normalize(glm::cross(camera.front, camera.up));
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        direction += glm::normalize(glm::cross(camera.front, camera.up));

    if (glm::length(direction) > 0)
        camera.move(direction, deltaTime);
}

void InputHandler::mouseCallback(GLFWwindow* window, double xpos, double ypos) {
    if (firstMouse) {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }

    float xOffset = xpos - lastX;
    float yOffset = lastY - ypos; // Reversed since y-coordinates range bottom to top
    lastX = xpos;
    lastY = ypos;

    Camera* cam = static_cast<Camera*>(glfwGetWindowUserPointer(window));
    if (cam) cam->rotate(xOffset, yOffset);
}
