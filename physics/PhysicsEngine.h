#ifndef PHYSICSENGINE_H
#define PHYSICSENGINE_H

#include <vector>
#include "RigidBody.h"

class PhysicsEngine {
public:
    std::vector<RigidBody>& getRigidBodies();

    void addRigidBody(const RigidBody& rigidBody);

private:
    std::vector<RigidBody> m_rigidBodies;
};

#endif // PHYSICSENGINE_H
