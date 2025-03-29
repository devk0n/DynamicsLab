#ifndef GRAVITY_BUILDER_H
#define GRAVITY_BUILDER_H

#include <memory>
#include "GravityForce.h"
#include "Proton.h"

namespace Proton {

class Dynamics;
class Body;

class GravityBuilder {
public:
  // Construct with a reference to the dynamics system and the gravity vector components.
  GravityBuilder(Dynamics &dynamics, double gx, double gy, double gz)
      : m_dynamics(dynamics),
        m_gravity(std::make_shared<GravityForce>(Vector3d(gx, gy, gz))) {}

  // Add a body to be influenced by gravity.
  GravityBuilder& addBody(Body* body) {
    m_gravity->addBody(body);
    return *this;
  }

  GravityBuilder& setGravity(double gx, double gy, double gz) {
    m_gravity->setGravity(Vector3d(gx, gy, gz));
    return *this;
  }

  // Finalize the builder: add the gravity force generator to the dynamics system.
  std::shared_ptr<GravityForce> build() {
    m_dynamics.addForceGenerator(m_gravity);
    return m_gravity;
  }

private:
  Dynamics &m_dynamics;
  std::shared_ptr<GravityForce> m_gravity;
};

} // Proton
#endif // GRAVITY_BUILDER_H
