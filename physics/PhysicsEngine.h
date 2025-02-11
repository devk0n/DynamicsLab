#ifndef PHYSICSENGINE_H
#define PHYSICSENGINE_H

#include <vector>
#include "RigidBody.h"

class PhysicsEngine {
public:
  PhysicsEngine() : PhysicsEngine(0.005) {}

  explicit PhysicsEngine(double timeStep);

  std::vector<RigidBody> &getRigidBodies();

  void addRigidBody(const RigidBody &rigidBody);

private:
  double m_timeStep;

  std::vector<RigidBody> m_rigidBodies;
};

#endif // PHYSICSENGINE_H
