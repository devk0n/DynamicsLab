#ifndef DISTANCE_CONSTRAINT_H
#define DISTANCE_CONSTRAINT_H

#include "Body.h"
#include "Constraint.h"

namespace Proton {

class DistanceConstraint final : public Constraint {
public:
  DistanceConstraint(
      Body* body1,
      Body* body2
  );

  void computePositionError(VectorXd& phi, int startRow) const override;
  void computeJacobian(MatrixXd& jacobian, int startRow) const override;
  void computeAccelerationCorrection(VectorXd& gamma, int startRow) const override;

private:
  Body* m_body1;
  Body* m_body2;
  double m_distance;

};
} // Proton

#endif // DISTANCE_CONSTRAINT_H
