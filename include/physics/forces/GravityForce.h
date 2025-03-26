#ifndef GRAVITY_FORCE_GENERATOR_H
#define GRAVITY_FORCE_GENERATOR_H

#include <ranges>

#include "Body.h"
#include "Proton.h"

#include "ForceGenerator.h"

namespace Proton {

class GravityForce final : public ForceGenerator {
public:
  explicit GravityForce(const Vector3d &gravity) : m_gravity(gravity) {}

  void addBody(Body* body) { m_targets.emplace(body->getID(), body); }

  void apply(double dt) override {
    for (const auto &body: m_targets | std::views::values) {
      body->addForce(body->getMass() * m_gravity);
    }
  }

private:
  Vector3d m_gravity{0.0, 0.0, -9.81};
  std::unordered_map<UniqueID, Body*> m_targets;
};

} // Proton

#endif // GRAVITY_FORCE_GENERATOR_H
