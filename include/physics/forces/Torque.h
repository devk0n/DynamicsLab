#ifndef TORQUE_H
#define TORQUE_H

#include <ranges>
#include "Body.h"
#include "Proton.h"
#include "ForceGenerator.h"

namespace Proton {
class Torque final : public ForceGenerator {
public:

  void addBody(Body* body) {
    m_targets.emplace(body->getID(), body);
  }

  void apply(double dt) override {
    for (const auto &body: m_targets | std::views::values) {
      if (body->getMass() <= 0.0) continue;
      body->addTorque(Vector3d(0, 0, 30));
    }
  }

private:
  std::unordered_map<UniqueID, Body*> m_targets;
};
} // Proton

#endif // TORQUE_H
