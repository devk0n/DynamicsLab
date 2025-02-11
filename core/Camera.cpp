#include "Camera.h"

// Constructor
Camera::Camera(glm::vec3 position, glm::vec3 up, float yaw, float pitch)
    : m_position(position), m_worldUp(up), m_yaw(yaw), m_pitch(pitch),
      m_front(glm::vec3(0.0f, 1.0f, 0.0f)), m_movementSpeed(2.5f), m_mouseSensitivity(0.1f) {
    updateCameraVectors();
}

// Get view matrix
glm::mat4 Camera::getViewMatrix() const {
    return glm::lookAt(m_position, m_position + m_front, m_up);
}

// Get projection matrix
glm::mat4 Camera::getProjectionMatrix(float aspectRatio, float near, float far) const {
    return glm::perspective(glm::radians(45.0f), aspectRatio, near, far);
}

// Move camera forward
void Camera::moveForward(float deltaTime) {
    m_position += m_front * m_movementSpeed * deltaTime;
}

// Move camera backward
void Camera::moveBackward(float deltaTime) {
    m_position -= m_front * m_movementSpeed * deltaTime;
}

// Move camera left
void Camera::moveLeft(float deltaTime) {
    m_position -= m_right * m_movementSpeed * deltaTime;
}

// Move camera right
void Camera::moveRight(float deltaTime) {
    m_position += m_right * m_movementSpeed * deltaTime;
}

// Move camera up
void Camera::moveUp(float deltaTime) {
    m_position += m_up * m_movementSpeed * deltaTime;
}


// Move camera down
void Camera::moveDown(float deltaTime) {
    m_position -= m_up * m_movementSpeed * deltaTime;
}

// Process mouse movement
void Camera::processMouseMovement(float xOffset, float yOffset, bool constrainPitch) {
    xOffset *= -m_mouseSensitivity;
    yOffset *= m_mouseSensitivity;

    m_yaw += xOffset;
    m_pitch += yOffset;

    // Constrain pitch to avoid flipping
    if (constrainPitch) {
        if (m_pitch > 89.0f) m_pitch = 89.0f;
        if (m_pitch < -89.0f) m_pitch = -89.0f;
    }

    // Update front, right, and up vectors
    updateCameraVectors();
}

// Get camera position
glm::vec3 Camera::getPosition() const {
    return m_position;
}

// Get camera front vector
glm::vec3 Camera::getFront() const {
    return m_front;
}

// Update camera vectors based on Euler angles
void Camera::updateCameraVectors() {
    // Calculate new front vector
    glm::vec3 front;
    front.x = cos(glm::radians(m_yaw)) * cos(glm::radians(m_pitch));
    front.y = sin(glm::radians(m_yaw)) * cos(glm::radians(m_pitch));
    front.z = sin(glm::radians(m_pitch));
    m_front = glm::normalize(front);

    // Recalculate right and up vectors
    m_right = glm::normalize(glm::cross(m_front, m_worldUp));
    m_up = glm::normalize(glm::cross(m_right, m_front));
}
