#ifndef DYNAMICSLAB_CAMERA_H
#define DYNAMICSLAB_CAMERA_H

#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"

class Camera {
public:
    Camera(float fov, float aspectRatio, float nearClip, float farClip);

    void setPosition(const glm::vec3& position);
    void setRotation(float yaw, float pitch);

    glm::mat4 getViewMatrix() const;
    glm::mat4 getProjectionMatrix() const;

private:
    glm::vec3 m_position;
    float m_yaw, m_pitch; // Rotation angles
    glm::mat4 m_projection;
};

#endif // DYNAMICSLAB_CAMERA_H
