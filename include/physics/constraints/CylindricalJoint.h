#ifndef CYLINDRICAL_JOINT_H
#define CYLINDRICAL_JOINT_H

#include "Body.h"
#include "Constraint.h"

namespace Proton {
class CylindricalJoint final : public Constraint {
public:
  CylindricalJoint() : Constraint(4) {}

  void computePositionError(VectorXd& phi, int startRow) const override;
  void computeJacobian(MatrixXd& jacobian, int startRow) const override;
  void computeAccelerationCorrection(VectorXd& gamma, int startRow) const override;

  [[nodiscard]] Body* getBodyA() const { return m_bodyA; }
  [[nodiscard]] Body* getBodyB() const { return m_bodyB; }
  [[nodiscard]] const Vector3d& getLocalPointA() const { return m_localPointA; }
  [[nodiscard]] const Vector3d& getLocalPointB() const { return m_localPointB; }
  [[nodiscard]] const Vector3d& getAxisA() const { return m_axisA; }
  [[nodiscard]] const Vector3d& getAxisB() const { return m_axisB; }
  [[nodiscard]] const Vector3d& getAxisM1() const { return m_axisM1; }
  [[nodiscard]] const Vector3d& getAxisN1() const { return m_axisN1; }
  [[nodiscard]] const Vector3d& getAxisM2() const { return m_axisM2; }
  [[nodiscard]] const Vector3d& getAxisN2() const { return m_axisN2; }

  void setBodyA(Body* bodyA) { m_bodyA = bodyA; }
  void setBodyB(Body* bodyB) { m_bodyB = bodyB; }
  void setLocalPointA(const Vector3d& localA) { m_localPointA = localA; }
  void setLocalPointB(const Vector3d& localB) { m_localPointB = localB; }
  void setAxisA(const Vector3d& axisA) { m_axisA = axisA; }
  void setAxisB(const Vector3d& axisB) { m_axisB = axisB; }
  void setAxisM1(const Vector3d& axisM1) { m_axisM1 = axisM1; }
  void setAxisN1(const Vector3d& axisN1) { m_axisN1 = axisN1; }
  void setAxisM2(const Vector3d& axisM2) { m_axisM2 = axisM2; }
  void setAxisN2(const Vector3d& axisN2) { m_axisN2 = axisN2; }

private:
  Body* m_bodyA{nullptr};
  Body* m_bodyB{nullptr};
  Vector3d m_localPointA{0, 0, 0};
  Vector3d m_localPointB{0, 0, 0};
  Vector3d m_axisA{0, 0, 0};
  Vector3d m_axisB{0, 0, 0};
  Vector3d m_axisM1{0, 0, 0};
  Vector3d m_axisM2{0, 0, 0};
  Vector3d m_axisN1{0, 0, 0};
  Vector3d m_axisN2{0, 0, 0};

};
} // namespace Proton

#endif // CYLINDRICAL_JOINT_H
