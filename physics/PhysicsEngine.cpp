#include "PhysicsEngine.h"

void PhysicsEngine::addRigidBody(const RigidBody &rigidBody) {
  m_rigidBodies.push_back(rigidBody);
}

std::vector<RigidBody> &PhysicsEngine::getRigidBodies() {
  return m_rigidBodies;
}
