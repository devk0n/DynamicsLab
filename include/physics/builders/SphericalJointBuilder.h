#ifndef SPHERICAL_JOINT_BUILDER_H
#define SPHERICAL_JOINT_BUILDER_H

#include "Dynamics.h"
#include "SphericalJoint.h"

namespace Proton {

class SphericalJointBuilder {
public:
  explicit SphericalJointBuilder(Dynamics& dynamics)
      : m_dynamics(dynamics),
        m_constraint(std::make_shared<SphericalJoint>()),
        m_autoDistance(true) {}

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

  SphericalJointBuilder& withDistance(double distance) {
    m_constraint->setDistance(distance);
    return *this;
  }

  SphericalJointBuilder& withAutoDistance(bool autoDistance) {
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
  Dynamics& m_dynamics;
  std::shared_ptr<SphericalJoint> m_constraint{nullptr};
  bool m_autoDistance;
};

} // namespace Proton

#endif // SPHERICAL_JOINT_BUILDER_H