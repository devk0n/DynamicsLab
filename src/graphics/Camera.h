#ifndef DYNAMICSLAB_CAMERA_H
#define DYNAMICSLAB_CAMERA_H

#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"

using namespace glm;

class Camera {
public:
    dvec3 position;
    dvec3 front;
    dvec3 up;

    double yaw, pitch;
    double speed, sensitivity;

    explicit Camera(dvec3 startPosition);

    dmat4 getViewMatrix();
    void move(const dvec3& direction, double deltaTime);
    void rotate(double xOffset, double yOffset);
};


#endif //DYNAMICSLAB_CAMERA_H
