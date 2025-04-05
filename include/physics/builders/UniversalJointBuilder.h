#ifndef UNIVERSAL_JOINT_BUILDER_H
#define UNIVERSAL_JOINT_BUILDER_H

#include "Dynamics.h"
#include "UniversalJoint.h"

namespace Proton {

class UniversalJointBuilder {
public:
  explicit UniversalJointBuilder(Dynamics& dynamics)
      : m_dynamics(dynamics),
        m_constraint(std::make_shared<UniversalJoint>()) {}

  UniversalJointBuilder& withBodyA(Body* body) {
    m_constraint->setBodyA(body);
    return *this;
  }

  UniversalJointBuilder& withBodyB(Body* body) {
    m_constraint->setBodyB(body);
    return *this;
  }

  UniversalJointBuilder& withLocalPointA(double x, double y, double z) {
    m_constraint->setLocalPointA(Vector3d(x, y, z));
    return *this;
  }

  UniversalJointBuilder& withLocalPointB(double x, double y, double z) {
    m_constraint->setLocalPointB(Vector3d(x, y, z));
    return *this;
  }

  UniversalJointBuilder& withAxisA(double x, double y, double z) {
    m_constraint->setAxisA(Vector3d(x, y, z));
    return *this;
  }

  UniversalJointBuilder& withAxisB(double x, double y, double z) {
    m_constraint->setAxisB(Vector3d(x, y, z));
    return *this;
  }

  std::shared_ptr<UniversalJoint> build() {
    m_dynamics.addConstraint(m_constraint);
    return m_constraint;
  }

private:
  Dynamics& m_dynamics;
  std::shared_ptr<UniversalJoint> m_constraint{nullptr};
};
} // Proton
#endif // UNIVERSAL_JOINT_BUILDER_H
