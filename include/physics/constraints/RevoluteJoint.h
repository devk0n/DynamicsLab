#ifndef REVOLUTE_JOINT_H
#define REVOLUTE_JOINT_H

#include "Body.h"
#include "Constraint.h"

namespace Proton {

class RevoluteJoint final : public Constraint {
public:
  RevoluteJoint(
      Body* body1, Vector3d local1, Vector3d axis1,
      Body* body2, Vector3d local2, Vector3d axis2, Vector3d axis3
  );

  void computePositionError(VectorXd &phi, int startRow) const override;
  void computeJacobian(MatrixXd &jacobian, int startRow) const override;
  void computeAccelerationCorrection(VectorXd &gamma, int startRow) const override;

  [[nodiscard]] Body* getBody1() const { return m_body1; }
  [[nodiscard]] Body* getBody2() const { return m_body2; }
  [[nodiscard]] Vector3d getLocal1() const { return m_local1; }
  [[nodiscard]] Vector3d getLocal2() const { return m_local2; }
  [[nodiscard]] Vector3d getAxis1() const { return m_axis1; }
  [[nodiscard]] Vector3d getAxis2() const { return m_axis2; }
  [[nodiscard]] Vector3d getAxis3() const { return m_axis3; }

private:
  Body* m_body1;
  Body* m_body2;
  Vector3d m_local1;
  Vector3d m_local2;
  Vector3d m_axis1;
  Vector3d m_axis2;
  Vector3d m_axis3;
};

} // Proton

#endif // REVOLUTE_JOINT_H
