#ifndef SPHERICAL_JOINT_BUILDER_H
#define SPHERICAL_JOINT_BUILDER_H

#include "Dynamics.h"
#include "SphericalJoint.h"

namespace Proton {

class SphericalJointBuilder {
public:
  explicit SphericalJointBuilder(Dynamics& dynamics)
      : m_dynamics(dynamics),
        m_constraint(std::make_shared<SphericalJoint>()) {}

  SphericalJointBuilder& between(Body* bodyA, Body* bodyB) {
    m_constraint->setBodyA(bodyA);
    m_constraint->setBodyB(bodyB);
    return *this;
  }

  SphericalJointBuilder& withBodyA(Body* body) {
    m_constraint->setBodyA(body);
    return *this;
  }

  SphericalJointBuilder& withBodyB(Body* body) {
    m_constraint->setBodyB(body);
    return *this;
  }

  SphericalJointBuilder& withLocalPointA(double x, double y, double z) {
    m_constraint->setLocalPointA(Vector3d(x, y, z));
    return *this;
  }

  SphericalJointBuilder& withLocalPointB(double x, double y, double z) {
    m_constraint->setLocalPointB(Vector3d(x, y, z));
    return *this;
  }

  std::shared_ptr<SphericalJoint> build() {
    m_dynamics.addConstraint(m_constraint);
    return m_constraint;
  }

private:
  Dynamics& m_dynamics;
  std::shared_ptr<SphericalJoint> m_constraint{nullptr};

};
} // namespace Proton
#endif // SPHERICAL_JOINT_BUILDER_H