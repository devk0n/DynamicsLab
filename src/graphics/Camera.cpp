#include "Camera.h"

Camera::Camera(
    const glm::vec3& position,
    float fov,
    float aspectRatio,
    float nearPlane,
    float farPlane
)
    : m_position(position),
      m_yaw(-90.0f),
      m_pitch(0.0f),
      m_fov(fov),
      m_aspectRatio(aspectRatio),
      m_nearPlane(nearPlane),
      m_farPlane(farPlane)
{
}

void Camera::setAspectRatio(float aspectRatio) {
    m_aspectRatio = aspectRatio;
}

void Camera::moveForward(float delta) {
    // Forward direction based on yaw/pitch
    glm::vec3 forward;
    forward.x = cos(glm::radians(m_yaw)) * cos(glm::radians(m_pitch));
    forward.y = sin(glm::radians(m_pitch));
    forward.z = sin(glm::radians(m_yaw)) * cos(glm::radians(m_pitch));
    m_position += glm::normalize(forward) * delta;
}

void Camera::moveRight(float delta) {
    // Right direction is cross between forward and world-up
    glm::vec3 forward;
    forward.x = cos(glm::radians(m_yaw)) * cos(glm::radians(m_pitch));
    forward.y = sin(glm::radians(m_pitch));
    forward.z = sin(glm::radians(m_yaw)) * cos(glm::radians(m_pitch));
    glm::vec3 right = glm::normalize(glm::cross(forward, glm::vec3(0.0f, 1.0f, 0.0f)));
    m_position += right * delta;
}

void Camera::moveUp(float delta) {
    // World up is (0,1,0)
    m_position.y += delta;
}

void Camera::rotatePitch(float angle) {
    m_pitch += angle;
    // Constrain pitch to avoid flipping
    if (m_pitch > 89.0f) m_pitch = 89.0f;
    if (m_pitch < -89.0f) m_pitch = -89.0f;
}

void Camera::rotateYaw(float angle) {
    m_yaw += angle;
}

glm::mat4 Camera::getViewMatrix() const {
    glm::vec3 forward;
    forward.x = cos(glm::radians(m_yaw)) * cos(glm::radians(m_pitch));
    forward.y = sin(glm::radians(m_pitch));
    forward.z = sin(glm::radians(m_yaw)) * cos(glm::radians(m_pitch));

    return glm::lookAt(m_position, m_position + glm::normalize(forward), glm::vec3(0.0f, 1.0f, 0.0f));
}

glm::mat4 Camera::getProjectionMatrix() const {
    return glm::perspective(glm::radians(m_fov), m_aspectRatio, m_nearPlane, m_farPlane);
}

glm::vec3 Camera::getPosition() const {
    return m_position;
}