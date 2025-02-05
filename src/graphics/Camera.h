//
// Created by devkon on 05/02/2025.
//

#ifndef DYNAMICSLAB_CAMERA_H
#define DYNAMICSLAB_CAMERA_H

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>


class Camera {
public:
    Camera();

    // Call once per frame to update the camera's orientation if right mouse is held
    void processMouseMovement(float mouseX, float mouseY, bool isRightMouseHeld);

    // Call once per frame to update the camera's position based on WASD input
    void processKeyboard(bool wKey, bool sKey, bool aKey, bool dKey, float deltaTime);

    // Get the view matrix (use in your renderer)
    [[nodiscard]] glm::mat4 getViewMatrix() const;

    // Position, orientation
    glm::vec3 position;
    float yaw;   // in degrees
    float pitch; // in degrees

private:
    float lastMouseX;
    float lastMouseY;
    bool firstMouse; // track whether the mouse just started moving

    // Cached direction vectors
    glm::vec3 front;
    glm::vec3 up;
    glm::vec3 rightVec;
    glm::vec3 worldUp;

    // Recompute the direction vectors based on yaw/pitch
    void updateCameraVectors();
};



#endif //DYNAMICSLAB_CAMERA_H
