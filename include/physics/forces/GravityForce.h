#ifndef GRAVITY_FORCE_GENERATOR_H
#define GRAVITY_FORCE_GENERATOR_H

#include "Body.h"
#include "Proton.h"

#include "ForceGenerator.h"

namespace Proton {

class GravityForce final : public ForceGenerator {
public:

  void addBody(Body* body) {
    m_targets.emplace(body->getID(), body);
  }

  void apply(double dt) override {
    for (const auto& [id, body] : m_targets) {
      if (body->getMass() <= 0.0) continue; // skip static/infinite mass bodies
      body->addForce(body->getMass() * m_gravity);
      // body->addTorque(Vector3d(10, 2, 1));
    }
  }

private:
  Vector3d m_gravity{0.0, 0.0, -9.81};
  std::unordered_map<UniqueID, Body*> m_targets;
};

} // Proton

#endif // GRAVITY_FORCE_GENERATOR_H
