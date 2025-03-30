#ifndef DYNAMICS_BUILDER_H
#define DYNAMICS_BUILDER_H

#include "Dynamics.h"
#include "BodyBuilder.h"
#include "SphericalJointBuilder.h"
#include "BallJointBuilder.h"
#include "GravityBuilder.h"
#include "SpringBuilder.h"

namespace Proton {

class DynamicsBuilder {
public:
  explicit DynamicsBuilder(Dynamics& dynamics) : m_dynamics(dynamics) {}

  // Creates a cube body and returns a fluent BodyBuilder
  [[nodiscard]] BodyBuilder createCube() const {
    UniqueID id = m_dynamics.addBody();
    Body* body = m_dynamics.getBody(id);
    return {m_dynamics, body};
  }

  [[nodiscard]] SphericalJointBuilder createSphericalJoint() const {
    return SphericalJointBuilder(m_dynamics);
  }

  [[nodiscard]] BallJointBuilder createBallJoint() const {
    return BallJointBuilder(m_dynamics);
  }

  [[nodiscard]] SpringBuilder createSpring() const {
    return SpringBuilder(m_dynamics);
  }

  [[nodiscard]] GravityBuilder createGravity(double gx = 0, double gy = 0, double gz = 0) const {
    return {m_dynamics, gx, gy, gz};
  }

private:
  Dynamics& m_dynamics;
};

} // namespace Proton

#endif // DYNAMICS_BUILDER_H