#ifndef REVOLUTE_JOINT_BUILDER_H
#define REVOLUTE_JOINT_BUILDER_H

#include "Dynamics.h"
#include "RevoluteJoint.h"

namespace Proton {

class RevoluteJointBuilder {
public:
  explicit RevoluteJointBuilder(Dynamics& dynamics)
      : m_dynamics(dynamics),
        m_constraint(std::make_shared<RevoluteJoint>()) {}

  RevoluteJointBuilder& withBodyA(Body* body) {
    m_constraint->setBodyA(body);
    return *this;
  }

  RevoluteJointBuilder& withBodyB(Body* body) {
    m_constraint->setBodyB(body);
    return *this;
  }

  RevoluteJointBuilder& withLocalPointA(double x, double y, double z) {
    m_constraint->setLocalPointA(Vector3d(x, y, z));
    return *this;
  }

  RevoluteJointBuilder& withLocalPointB(double x, double y, double z) {
    m_constraint->setLocalPointB(Vector3d(x, y, z));
    return *this;
  }

  RevoluteJointBuilder& withAxisA(double x, double y, double z) {
    m_constraint->setAxisA(Vector3d(x, y, z));
    return *this;
  }

  RevoluteJointBuilder& withAxisB(double x, double y, double z) {
    m_constraint->setAxisB(Vector3d(x, y, z));
    return *this;
  }

  RevoluteJointBuilder& withAxisC(double x, double y, double z) {
    m_constraint->setAxisC(Vector3d(x, y, z));
    return *this;
  }

  std::shared_ptr<RevoluteJoint> build() {
    m_dynamics.addConstraint(m_constraint);
    return m_constraint;
  }

private:
  Dynamics& m_dynamics;
  std::shared_ptr<RevoluteJoint> m_constraint;
};
} // Proton
#endif // REVOLUTE_JOINT_BUILDER_H
