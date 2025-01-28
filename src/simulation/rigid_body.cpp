//
// Created by devkon on 28/01/2025.
//

#include "rigid_body.h"

RigidBody::RigidBody(const Eigen::Matrix<double, 7, 1>& initialState,
                     const Eigen::Matrix<double, 7, 1>& initialVelocity)
    : state(initialState), velocity(initialVelocity) {}
