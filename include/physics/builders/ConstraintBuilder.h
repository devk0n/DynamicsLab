#ifndef CONSTRAINT_BUILDER_H
#define CONSTRAINT_BUILDER_H

#include "Dynamics.h"

namespace Proton {

// Generic template for constraints without extra parameters.
template <typename T>
class ConstraintBuilder {
public:
  explicit ConstraintBuilder(Dynamics &dynamics)
        : m_dynamics(dynamics),
          m_constraint(std::make_shared<T>()) {}

private:
  Dynamics &m_dynamics;
  std::shared_ptr<T> m_constraint;
};

template <>
class ConstraintBuilder<BallJoint> {
public:
  explicit ConstraintBuilder(Dynamics &dynamics)
      : m_dynamics(dynamics),
        m_constraint(std::make_shared<BallJoint>()) {}

  ConstraintBuilder& withBodyA(Body* body) {
    m_constraint->setBodyA(body);
    return *this;
  }

  ConstraintBuilder& withBodyB(Body* body) {
    m_constraint->setBodyB(body);
    return *this;
  }

  ConstraintBuilder& withLocalPointA(double x, double y, double z) {
    m_constraint->setLocalPointA(Vector3d(x, y, z));
    return *this;
  }

  ConstraintBuilder& withLocalPointB(double x, double y, double z) {
    m_constraint->setLocalPointB(Vector3d(x, y, z));
    return *this;
  }

  std::shared_ptr<BallJoint> build() {
    m_dynamics.addConstraint(m_constraint);
    return m_constraint;
  }

private:
  Dynamics &m_dynamics;
  std::shared_ptr<BallJoint> m_constraint;
};

template <>
class ConstraintBuilder<SphericalJoint> {
public:
  explicit ConstraintBuilder(Dynamics &dynamics)
      : m_dynamics(dynamics),
        m_constraint(std::make_shared<SphericalJoint>()) {}

  ConstraintBuilder& withBodyA(Body* body) {
    m_constraint->setBodyA(body);
    return *this;
  }

  ConstraintBuilder& withBodyB(Body* body) {
    m_constraint->setBodyB(body);
    return *this;
  }

  ConstraintBuilder& withLocalPointA(double x, double y, double z) {
    m_constraint->setLocalPointA(Vector3d(x, y, z));
    return *this;
  }

  ConstraintBuilder& withLocalPointB(double x, double y, double z) {
    m_constraint->setLocalPointB(Vector3d(x, y, z));
    return *this;
  }

  ConstraintBuilder& withDistance(double distance) {
    m_constraint->setDistance(distance);
    return *this;
  }

  ConstraintBuilder& withAutoDistance(bool autoDistance) {
    m_autoDistance = autoDistance;
    return *this;
  }

  std::shared_ptr<SphericalJoint> build() {
    if (m_autoDistance) {
      m_constraint->computeDistance();
    }
    m_dynamics.addConstraint(m_constraint);
    return m_constraint;
  }

private:
  Dynamics &m_dynamics;
  std::shared_ptr<SphericalJoint> m_constraint;
  bool m_autoDistance = true;
};

} // Proton

#endif // CONSTRAINT_BUILDER_H
