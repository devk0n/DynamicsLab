#ifndef DYNAMICS_BUILDER_H
#define DYNAMICS_BUILDER_H

#include "Dynamics.h"
#include "BodyBuilder.h"
#include "ConstraintBuilder.h"
#include "GravityBuilder.h"

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

  template<typename ConstraintType>
  ConstraintBuilder<ConstraintType> createConstraint() {
    return ConstraintBuilder<ConstraintType>(m_dynamics);
  }

  [[nodiscard]] GravityBuilder createGravity(double gx, double gy, double gz) const {
    return {m_dynamics, gx, gy, gz};
  }

  [[nodiscard]] GravityBuilder createGravity() const {
    return {m_dynamics, 0, 0, 0};
  }

private:
  Dynamics& m_dynamics;

};
} // Proton
#endif // DYNAMICS_BUILDER_H
