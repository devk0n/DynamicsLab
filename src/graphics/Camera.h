#ifndef DYNAMICSLAB_CAMERA_H
#define DYNAMICSLAB_CAMERA_H

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

class Camera {
public:
    explicit Camera(
        const glm::vec3& position = glm::vec3(0.0f, 0.0f, 3.0f),
        float fov = 45.0f,
        float aspectRatio = 16.0f / 9.0f,
        float nearPlane = 0.1f,
        float farPlane = 100.0f
    );

    void setAspectRatio(float aspectRatio);
    void moveForward(float delta);
    void moveRight(float delta);
    void moveUp(float delta);
    void rotatePitch(float angle);
    void rotateYaw(float angle);

    glm::mat4 getViewMatrix() const;
    glm::mat4 getProjectionMatrix() const;
    glm::vec3 getPosition() const;

private:
    glm::vec3 m_position;
    float m_yaw;   // left-right rotation
    float m_pitch; // up-down rotation

    float m_fov;
    float m_aspectRatio;
    float m_nearPlane;
    float m_farPlane;
};

#endif //DYNAMICSLAB_CAMERA_H