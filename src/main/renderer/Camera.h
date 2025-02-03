#ifndef DYNAMICSLAB_CAMERA_H
#define DYNAMICSLAB_CAMERA_H

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

class Camera {
public:
    glm::vec3 position;
    glm::vec3 front;
    glm::vec3 up;

    float yaw, pitch;
    float speed, sensitivity;

    explicit Camera(glm::vec3 startPosition);

    glm::mat4 getViewMatrix();
    void move(const glm::vec3& direction, float deltaTime);
    void rotate(float xOffset, float yOffset);
};


#endif //DYNAMICSLAB_CAMERA_H
