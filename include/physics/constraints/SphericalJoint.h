#ifndef SPHERICAL_JOINT_H
#define SPHERICAL_JOINT_H

#include "Body.h"
#include "Constraint.h"

namespace Proton {

class SphericalJoint final : public Constraint {
public:
  SphericalJoint(
      Body* body1,
      Vector3d local1,
      Body* body2,
      Vector3d local2
  );

  void computePositionError(VectorXd& phi, int startRow) const override;
  void computeJacobian(MatrixXd& jacobian, int startRow) const override;
  void computeAccelerationCorrection(VectorXd& gamma, int startRow) const override;

  [[nodiscard]] Body* getBody1() const { return m_body1; }
  [[nodiscard]] Body* getBody2() const { return m_body2; }
  [[nodiscard]] Vector3d getLocal1() const { return m_local1; }
  [[nodiscard]] Vector3d getLocal2() const { return m_local2; }

private:
  Body* m_body1;
  Body* m_body2;
  Vector3d m_local1;
  Vector3d m_local2;
};
} // Proton

#endif // SPHERICAL_JOINT_H
