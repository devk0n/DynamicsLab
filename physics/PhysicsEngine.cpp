#include "PhysicsEngine.h"

PhysicsEngine::PhysicsEngine(double timeStep) : m_timeStep(timeStep) {

}

void PhysicsEngine::addRigidBody(const RigidBody &rigidBody) {
  m_rigidBodies.push_back(rigidBody);
}

std::vector<RigidBody> &PhysicsEngine::getRigidBodies() {
  return m_rigidBodies;
}
