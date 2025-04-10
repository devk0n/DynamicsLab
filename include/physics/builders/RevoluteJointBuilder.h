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

  RevoluteJointBuilder& between(Body* bodyA, Body* bodyB) {
    m_constraint->setBodyA(bodyA);
    m_constraint->setBodyB(bodyB);
    return *this;
  }

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
    m_constraint->setAxisA(Vector3d(x, y, z).normalized());
    return *this;
  }

  RevoluteJointBuilder& withAxisB(double x, double y, double z) {
    m_constraint->setAxisB(Vector3d(x, y, z).normalized());
    return *this;
  }

  RevoluteJointBuilder& withAxisM(double x, double y, double z) {
    m_constraint->setAxisM(Vector3d(x, y, z).normalized());
    return *this;
  }

  RevoluteJointBuilder& withAxis(double x, double y, double z) {
    const Vector3d axis = Vector3d(x, y, z).normalized();

    m_constraint->setAxisA(axis);
    m_constraint->setAxisB(axis);

    // Auto-generate orthonormal basis (M, N) for internal constraint calculations
    Vector3d arbitrary = std::abs(axis.x()) < 0.9 ? Vector3d::UnitX() : Vector3d::UnitY();
    Vector3d M = axis.cross(arbitrary).normalized();
    Vector3d N = axis.cross(M).normalized();

    m_constraint->setAxisM(M);
    m_constraint->setAxisN(N);

    return *this;
  }

  RevoluteJointBuilder& withAxisN(double x, double y, double z) {
    m_constraint->setAxisN(Vector3d(x, y, z).normalized());
    return *this;
  }

  std::shared_ptr<RevoluteJoint> build() {
    // m_constraint->calculateConstraintAxes();
    m_dynamics.addConstraint(m_constraint);
    return m_constraint;
  }

private:
  Dynamics& m_dynamics;
  std::shared_ptr<RevoluteJoint> m_constraint{nullptr};
};
} // Proton
#endif // REVOLUTE_JOINT_BUILDER_H
