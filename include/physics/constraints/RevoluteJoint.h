#ifndef REVOLUTE_JOINT_H
#define REVOLUTE_JOINT_H

#include "Body.h"
#include "Constraint.h"

namespace Proton {

class RevoluteJoint final : public Constraint {
public:
  RevoluteJoint(
      Body* bodyA, Vector3d localPointA, Vector3d axisA,
      Body* bodyB, Vector3d localPointB, Vector3d axisB, Vector3d axisC
  );

  void computePositionError(VectorXd &phi, int startRow) const override;
  void computeJacobian(MatrixXd &jacobian, int startRow) const override;
  void computeAccelerationCorrection(VectorXd &gamma, int startRow) const override;

  [[nodiscard]] Body* getBodyA() const { return m_bodyA; }
  [[nodiscard]] Body* getBodyB() const { return m_bodyB; }
  [[nodiscard]] Vector3d getLocalPointA() const { return m_localPointA; }
  [[nodiscard]] Vector3d getLocalPointB() const { return m_localPointB; }
  [[nodiscard]] Vector3d getAxisA() const { return m_axisA; }
  [[nodiscard]] Vector3d getAxisB() const { return m_axisB; }
  [[nodiscard]] Vector3d getAxisC() const { return m_axisC; }



private:
  Body* m_bodyA;
  Body* m_bodyB;
  Vector3d m_localPointA;
  Vector3d m_localPointB;
  Vector3d m_axisA;
  Vector3d m_axisB;
  Vector3d m_axisC;
};

} // Proton

#endif // REVOLUTE_JOINT_H
