#include "Camera.h"

Camera::Camera(float fov, float aspectRatio, float nearClip, float farClip)
    : m_position(0.0f, 0.0f, 3.0f), m_yaw(0.0f), m_pitch(0.0f) {
    m_projection = glm::perspective(glm::radians(fov), aspectRatio, nearClip, farClip);
}

void Camera::setPosition(const glm::vec3& position) {
    m_position = position;
}

void Camera::setRotation(float yaw, float pitch) {
    m_yaw = yaw;
    m_pitch = pitch;
}

glm::mat4 Camera::getViewMatrix() const {
    glm::vec3 front;
    front.x = cos(glm::radians(m_yaw)) * cos(glm::radians(m_pitch));
    front.y = sin(glm::radians(m_pitch));
    front.z = sin(glm::radians(m_yaw)) * cos(glm::radians(m_pitch));
    front = glm::normalize(front);

    return glm::lookAt(m_position, m_position + front, glm::vec3(0.0f, 1.0f, 0.0f));
}

glm::mat4 Camera::getProjectionMatrix() const {
    return m_projection;
}
