#ifndef DYNAMICSLAB_RIGID_BODY_H
#define DYNAMICSLAB_RIGID_BODY_H

#include <Eigen/Dense>

class RigidBody {
public:
    RigidBody(const Eigen::Matrix<double, 7, 1>& initialState, const Eigen::Matrix<double, 7, 1>& initialVelocity);

private:
    // State vector: [position(3), quaternion(4)]
    Eigen::Vector<double, 7> state;

    // Velocity vector: [linear(3), angular(4)]
    Eigen::Vector<double, 7> velocity;
};


#endif //DYNAMICSLAB_RIGID_BODY_H
