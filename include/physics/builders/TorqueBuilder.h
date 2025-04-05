#ifndef TORQUE_BUILDER_H
#define TORQUE_BUILDER_H

#include <Dynamics.h>
#include <memory>
#include "Torque.h"
#include "Proton.h"

namespace Proton {
class TorqueBuilder {
public:
  TorqueBuilder(Dynamics &dynamics, double gx, double gy, double gz)
      : m_dynamics(dynamics),
        m_torque(std::make_shared<Torque>(Vector3d(gx, gy, gz))) {}

  // Add a body to be influenced by Torque.
  TorqueBuilder& addBody(Body* body) {
    m_torque->addBody(body);
    return *this;
  }

  TorqueBuilder& setGravity(double gx, double gy, double gz) {
    m_torque->setTorque(Vector3d(gx, gy, gz));
    return *this;
  }

  // Finalize the builder: add the gravity force generator to the dynamics system.
  std::shared_ptr<Torque> build() {
    m_dynamics.addForceGenerator(m_torque);
    return m_torque;
  }

private:
  Dynamics &m_dynamics;
  std::shared_ptr<Torque> m_torque{nullptr};

};
} // Proton

#endif // TORQUE_BUILDER_H
