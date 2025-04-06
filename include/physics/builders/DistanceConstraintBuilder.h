#ifndef DISTANCE_CONSTRAINT_BUILDER_H
#define DISTANCE_CONSTRAINT_BUILDER_H

#include "Dynamics.h"
#include "DistanceConstraint.h"

namespace Proton {

class DistanceConstraintBuilder {
public:
  explicit DistanceConstraintBuilder(Dynamics& dynamics)
      : m_dynamics(dynamics),
        m_constraint(std::make_shared<DistanceConstraint>()) {}

  DistanceConstraintBuilder& withBodyA(Body* body) {
    m_constraint->setBodyA(body);
    return *this;
  }

  DistanceConstraintBuilder& withBodyB(Body* body) {
    m_constraint->setBodyB(body);
    return *this;
  }

  DistanceConstraintBuilder& withLocalPointA(double x, double y, double z) {
    m_constraint->setLocalPointA(Vector3d(x, y, z));
    return *this;
  }

  DistanceConstraintBuilder& withLocalPointB(double x, double y, double z) {
    m_constraint->setLocalPointB(Vector3d(x, y, z));
    return *this;
  }

  DistanceConstraintBuilder& withDistance(double distance) {
    m_constraint->setDistance(distance);
    return *this;
  }

  DistanceConstraintBuilder& withAutoDistance(bool autoDistance) {
    m_autoDistance = autoDistance;
    return *this;
  }

  std::shared_ptr<DistanceConstraint> build() {
    if (m_autoDistance) {
      m_constraint->computeDistance();
    }
    m_dynamics.addConstraint(m_constraint);
    return m_constraint;
  }

private:
  Dynamics& m_dynamics;
  std::shared_ptr<DistanceConstraint> m_constraint{nullptr};
  bool m_autoDistance{true};
};
} // namespace Proton
#endif // DISTANCE_CONSTRAINT_BUILDER_H