#ifndef GRAVITY_BUILDER_H
#define GRAVITY_BUILDER_H

#include <memory>
#include "Gravity.h"
#include "Proton.h"

namespace Proton {
class GravityBuilder {
public:
  GravityBuilder(
    Dynamics& dynamics,
    const double& gx,
    const double& gy,
    const double& gz)
      : m_dynamics(dynamics),
        m_gravity(std::make_shared<Gravity>(Vector3d(gx, gy, gz))) {}

  GravityBuilder& addBody(Body* body) {
    m_gravity->addBody(body);
    return *this;
  }

  GravityBuilder& setGravity(const double& gx, const double& gy, const double& gz) {
    m_gravity->setGravity(Vector3d(gx, gy, gz));
    return *this;
  }

  std::shared_ptr<Gravity> build() {
    m_dynamics.addForceGenerator(m_gravity);
    return m_gravity;
  }

private:
  Dynamics& m_dynamics;
  std::shared_ptr<Gravity> m_gravity{nullptr};

};
} // namespace Proton
#endif // GRAVITY_BUILDER_H
