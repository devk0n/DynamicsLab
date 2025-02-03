#include "Camera.h"

Camera::Camera(glm::dvec3 startPosition)
    : position(startPosition), front(glm::dvec3(0.0, 0.0, -1.0)), up(glm::dvec3(0.0, 1.0, 0.0)),
      yaw(-90.0), pitch(0.0), speed(2.5), sensitivity(0.1) {}

glm::dmat4 Camera::getViewMatrix() {
    return glm::lookAt(position, position + front, up);
}

void Camera::move(const glm::dvec3& direction, double deltaTime) {
    double velocity = speed * deltaTime;
    position += direction * velocity;
}

void Camera::rotate(double xOffset, double yOffset) {
    xOffset *= sensitivity;
    yOffset *= sensitivity;

    yaw += xOffset;
    pitch -= yOffset;

    if (pitch > 89.0f) pitch = 89.0f;
    if (pitch < -89.0f) pitch = -89.0f;

    glm::dvec3 direction;
    direction.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
    direction.y = sin(glm::radians(pitch));
    direction.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
    front = glm::normalize(direction);
}
