#ifndef UNIVERSAL_JOINT_H
#define UNIVERSAL_JOINT_H

#include "Body.h"
#include "Constraint.h"

namespace Proton {
class UniversalJoint final : public Constraint {
public:
  UniversalJoint() : Constraint(4) {}

  void computePositionError(VectorXd& phi, int startRow) const override;
  void computeJacobian(MatrixXd& jacobian, int startRow) const override;
  void computeAccelerationCorrection(VectorXd& gamma, int startRow) const override;

  [[nodiscard]] Body* getBodyA() const { return m_bodyA; }
  [[nodiscard]] Body* getBodyB() const { return m_bodyB; }
  [[nodiscard]] const Vector3d& getLocalPointA() const { return m_localPointA; }
  [[nodiscard]] const Vector3d& getLocalPointB() const { return m_localPointB; }
  [[nodiscard]] const Vector3d& getAxisA() const { return m_axisA; }
  [[nodiscard]] const Vector3d& getAxisB() const { return m_axisB; }

  void setBodyA(Body* bodyA) { m_bodyA = bodyA; }
  void setBodyB(Body* bodyB) { m_bodyB = bodyB; }
  void setLocalPointA(const Vector3d& localA) { m_localPointA = localA; }
  void setLocalPointB(const Vector3d& localB) { m_localPointB = localB; }
  void setAxisA(const Vector3d& axisA) { m_axisA = axisA; }
  void setAxisB(const Vector3d& axisB) { m_axisB = axisB; }

private:
  Body* m_bodyA{nullptr};
  Body* m_bodyB{nullptr};
  Vector3d m_localPointA{0.0, 0.0, 0.0};
  Vector3d m_localPointB{0.0, 0.0, 0.0};
  Vector3d m_axisA{0.0, 0.0, 0.0};
  Vector3d m_axisB{0.0, 0.0, 0.0};
};
} // Proton
#endif // UNIVERSAL_JOINT_H
