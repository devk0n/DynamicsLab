#ifndef GRAVITY_FORCE_GENERATOR_H
#define GRAVITY_FORCE_GENERATOR_H

#include <ranges>
#include <utility>

#include "Body.h"
#include "Proton.h"
#include "ForceGenerator.h"

namespace Proton {

class GravityForce final : public ForceGenerator {
public:
  explicit GravityForce(Vector3d gravity) : m_gravity(std::move(gravity)) {}

  void addBody(Body* body) { m_targets.emplace(body->getID(), body); }

  [[nodiscard]] const Vector3d& getGravity() const { return m_gravity; }
  void setGravity(const Vector3d& gravity) { m_gravity = gravity; }

  void apply(double dt) override {
    if (m_targets.empty() || m_gravity.norm() == 0) return;
    for (const auto &body : m_targets | std::views::values) {
      Vector3d force = body->getMass() * m_gravity;
      body->addForce(force);
    }
  }

private:
  Vector3d m_gravity{0.0, 0.0, 0.0};
  std::unordered_map<UniqueID, Body*> m_targets{};
};

} // Proton

#endif // GRAVITY_FORCE_GENERATOR_H
