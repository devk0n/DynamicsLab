#ifndef SPHERICAL_JOINT_H
#define SPHERICAL_JOINT_H

#include "Body.h"
#include "Constraint.h"

namespace Proton {

class SphericalJoint : public Constraint {
public:
  SphericalJoint(
      Body* body1,
      const Vector3d &local1,
      Body* body2,
      const Vector3d &local2
  );

  void computePositionError(VectorXd& phi, int startRow) const override;
  void computeJacobian(MatrixXd& jacobian, int startRow) const override;
  void computeAccelerationCorrection(VectorXd& gamma, int startRow) const override;

private:
  Body* m_body1;
  Body* m_body2;
  Vector3d m_local1;
  Vector3d m_local2;
  double m_distance;
};
} // Proton

#endif // SPHERICAL_JOINT_H
