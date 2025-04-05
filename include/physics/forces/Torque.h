#ifndef TORQUE_H
#define TORQUE_H

#include <ranges>
#include "Body.h"
#include "Proton.h"
#include "ForceGenerator.h"

namespace Proton {
class Torque final : public ForceGenerator {
public:
  explicit Torque(Vector3d torque) : m_torque(std::move(torque)) {}

  void addBody(Body* body) { m_targets.emplace(body->getID(), body); }

  [[nodiscard]] const Vector3d& getTorque() const { return m_torque; }
  void setTorque(const Vector3d& torque) { m_torque = torque; }

  void apply(double dt) override {
    if (m_targets.empty() || m_torque.norm() == 0) return;
    for (const auto &body : m_targets | std::views::values) {
      body->addTorque(m_torque);
    }
  }

private:
  Vector3d m_torque{0.0, 0.0, 0.0};
  std::unordered_map<UniqueID, Body*> m_targets{};
};
} // Proton

#endif // TORQUE_H
