#ifndef UNIVERSAL_JOINT_H
#define UNIVERSAL_JOINT_H

#include <utility>

#include "Body.h"
#include "Constraint.h"

namespace Proton {
class UniversalJoint final : public Constraint {
public:
  UniversalJoint(
      Body* bodyA, Vector3d localPointA, Vector3d axisA,
      Body* bodyB, Vector3d localPointB, Vector3d axisB
  );

  void computePositionError(VectorXd& phi, int startRow) const override;
  void computeJacobian(MatrixXd& jacobian, int startRow) const override;
  void computeAccelerationCorrection(VectorXd& gamma, int startRow) const override;

  [[nodiscard]] Body* getBodyA() const { return m_bodyA; }
  [[nodiscard]] Body* getBodyB() const { return m_bodyB; }
  [[nodiscard]] Vector3d getLocalPointA() const { return m_localPointA; }
  [[nodiscard]] Vector3d getLocalPointB() const { return m_localPointB; }
  [[nodiscard]] Vector3d getAxisA() const { return m_axisA; }
  [[nodiscard]] Vector3d getAxisB() const { return m_axisB; }

  void setBodyA(Body* bodyA) { m_bodyA = bodyA; }
  void setBodyB(Body* bodyB) { m_bodyB = bodyB; }
  void setlocalPointA(Vector3d localA) { m_localPointA = std::move(localA); }
  void setLocalPointB(Vector3d localB) { m_localPointB = std::move(localB); }
  void setAxisA(Vector3d axisA) { m_axisA = std::move(axisA); }
  void setAxisB(Vector3d axisB) { m_axisB = std::move(axisB); }

private:
  Body* m_bodyA;
  Body* m_bodyB;
  Vector3d m_localPointA;
  Vector3d m_localPointB;
  Vector3d m_axisA;
  Vector3d m_axisB;

};
} // Proton

#endif // UNIVERSAL_JOINT_H
