#include "Dynamics.h"

Dynamics::Dynamics(double stepSize) : m_stepSize(stepSize) {
}

void Dynamics::addRigidBody(std::unique_ptr<RigidBody> rigidBody) {
    m_rigidBodies.push_back(std::move(rigidBody));
}
