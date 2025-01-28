//
// Created by devkon on 28/01/2025.
//

#ifndef DYNAMICSLAB_DYNAMICS_H
#define DYNAMICSLAB_DYNAMICS_H

#include <Eigen/Dense>

struct RidgedBody {
    double mass;
    double length, height, width;
    double xInertia, yInertia, zInertia;
    double xPosition, yPosition, zPosition;
    double xVelocity, yVelocity, zVelocity;
    double wOrientation, xOrientation, yOrientation, zOrientation;
    double w
};

class dynamics {
public:

private:

};


#endif //DYNAMICSLAB_DYNAMICS_H
