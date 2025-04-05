#ifndef BALL_JOINT_BUILDER_H
#define BALL_JOINT_BUILDER_H

#include "Dynamics.h"
#include "BallJoint.h"

namespace Proton {

class BallJointBuilder {
public:
  explicit BallJointBuilder(Dynamics& dynamics)
      : m_dynamics(dynamics),
        m_constraint(std::make_shared<BallJoint>()) {}

  BallJointBuilder& withBodyA(Body* body) {
    m_constraint->setBodyA(body);
    return *this;
  }

  BallJointBuilder& withBodyB(Body* body) {
    m_constraint->setBodyB(body);
    return *this;
  }

  BallJointBuilder& withLocalPointA(double x, double y, double z) {
    m_constraint->setLocalPointA(Vector3d(x, y, z));
    return *this;
  }

  BallJointBuilder& withLocalPointB(double x, double y, double z) {
    m_constraint->setLocalPointB(Vector3d(x, y, z));
    return *this;
  }

  std::shared_ptr<BallJoint> build() {
    m_dynamics.addConstraint(m_constraint);
    return m_constraint;
  }

private:
  Dynamics& m_dynamics;
  std::shared_ptr<BallJoint> m_constraint{nullptr};
};

} // namespace Proton

#endif // BALL_JOINT_BUILDER_H