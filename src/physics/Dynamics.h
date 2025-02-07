#ifndef DYNAMICSLAB_DYNAMICS_H
#define DYNAMICSLAB_DYNAMICS_H

#include <vector>
#include <memory>
#include <Eigen/Dense>
#include "RigidBody.h"

class Dynamics {
public:
    Dynamics(double stepSize);
    void addRigidBody(std::unique_ptr<RigidBody> rigidBody);

private:
    std::vector<std::unique_ptr<RigidBody>> m_rigidBodies;

    double m_stepSize;
};


#endif
