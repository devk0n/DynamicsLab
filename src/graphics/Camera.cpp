#include "Camera.h"
#include "input/InputManager.h"


Camera::Camera()
    : position(0.0f, 3.0f, 5.0f) // Start a bit away from origin
    , yaw(-90.0f)               // Point down -Z by default
    , pitch(-30.0f)
    , lastMouseX(0.0f)
    , lastMouseY(0.0f)
    , firstMouse(true)
    , front(0.0f, 0.0f, -1.0f)
    , up(0.0f, 1.0f, 0.0f)
    , rightVec(1.0f, 0.0f, 0.0f)
    , worldUp(0.0f, 1.0f, 0.0f) {
    updateCameraVectors();
}

// Helper to recalc front, right, up from yaw/pitch
void Camera::updateCameraVectors() {
    // Convert angles to radians
    float radYaw   = glm::radians(yaw);
    float radPitch = glm::radians(pitch);

    // Calculate the new Front vector
    glm::vec3 newFront;
    newFront.x = cos(radYaw) * cos(radPitch);
    newFront.y = sin(radPitch);
    newFront.z = sin(radYaw) * cos(radPitch);
    front = glm::normalize(newFront);

    // Re-calc Right and Up
    rightVec = glm::normalize(glm::cross(front, worldUp));
    up       = glm::normalize(glm::cross(rightVec, front));
}

// Mouse movement
void Camera::processMouseMovement(float mouseX, float mouseY, bool isRightMouseHeld) {
    if (!isRightMouseHeld) {
        // If right mouse button is NOT held, we skip updating the camera
        firstMouse = true; // next time it’s pressed, we reset
        return;
    }

    // On first click, just store initial mouse coords to avoid a large jump
    if (firstMouse) {
        lastMouseX = mouseX;
        lastMouseY = mouseY;
        firstMouse = false;
    }

    // Calculate offset
    float xoffset = mouseX - lastMouseX;
    float yoffset = lastMouseY - mouseY; // reversed since y-coord goes from top to bottom

    lastMouseX = mouseX;
    lastMouseY = mouseY;

    // Adjust sensitivity to taste
    float sensitivity = 0.1f;
    xoffset *= sensitivity;
    yoffset *= sensitivity;

    yaw   += xoffset;
    pitch += yoffset;

    // Constrain the pitch
    if (pitch > 89.0f)  pitch = 89.0f;
    if (pitch < -89.0f) pitch = -89.0f;

    // Recompute direction vectors
    updateCameraVectors();
}

void Camera::processKeyboard(const KeyBindings& keys, float deltaTime) {
    float speed = 25.0f * deltaTime; // movement speed

    if (InputManager::isKeyPressed(keys.moveForward)) {
        position += front * speed;
    }
    if (InputManager::isKeyPressed(keys.moveBackward)) {
        position -= front * speed;
    }
    if (InputManager::isKeyPressed(keys.moveLeft)) {
        position -= rightVec * speed;
    }
    if (InputManager::isKeyPressed(keys.moveRight)) {
        position += rightVec * speed;
    }
    if (InputManager::isKeyPressed(keys.moveUp)) {
        position += up * speed;
    }
    if (InputManager::isKeyPressed(keys.moveDown)) {
        position -= up * speed;
    }
}

glm::mat4 Camera::getViewMatrix() const {
    return glm::lookAt(position, position + front, up);
}
