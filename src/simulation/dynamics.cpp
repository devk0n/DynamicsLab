#include "dynamics.h"

void Dynamics::addBody(std::shared_ptr<RigidBody> body) {
    m_Bodies.push_back(body);
}