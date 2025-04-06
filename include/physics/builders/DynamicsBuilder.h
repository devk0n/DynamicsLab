#ifndef DYNAMICS_BUILDER_H
#define DYNAMICS_BUILDER_H

#include "Dynamics.h"
#include "BodyBuilder.h"
#include "SphericalJointBuilder.h"
#include "DistanceConstraintBuilder.h"
#include "GravityBuilder.h"
#include "SpringBuilder.h"
#include "UniversalJointBuilder.h"
#include "RevoluteJointBuilder.h"
#include "TorqueBuilder.h"

namespace Proton {

class DynamicsBuilder {
public:
  explicit DynamicsBuilder(Dynamics& dynamics) : m_dynamics(dynamics) {}

  [[nodiscard]] BodyBuilder createCube() const {
    UniqueID id = m_dynamics.addBody();
    Body* body = m_dynamics.getBody(id);
    return {m_dynamics, body};
  }

  [[nodiscard]] DistanceConstraintBuilder createDistanceConstraint() const {
    return DistanceConstraintBuilder(m_dynamics);
  }

  [[nodiscard]] SphericalJointBuilder createSphericalJoint() const {
    return SphericalJointBuilder(m_dynamics);
  }

  [[nodiscard]] UniversalJointBuilder createUniversalJoint() const {
    return UniversalJointBuilder(m_dynamics);
  }

  [[nodiscard]] RevoluteJointBuilder createRevoluteJoint() const {
    return RevoluteJointBuilder(m_dynamics);
  }

  [[nodiscard]] SpringBuilder createSpring() const {
    return SpringBuilder(m_dynamics);
  }

  [[nodiscard]] GravityBuilder createGravity(const double& gx = 0, const double& gy = 0, const double& gz = 0) const {
    return {m_dynamics, gx, gy, gz};
  }

  [[nodiscard]] TorqueBuilder createTorque(const double& tx = 0, const double& ty = 0, const double& tz = 0) const {
    return {m_dynamics, tx, ty, tz};
  }

private:
  Dynamics& m_dynamics;

};
} // namespace Proton
#endif // DYNAMICS_BUILDER_H