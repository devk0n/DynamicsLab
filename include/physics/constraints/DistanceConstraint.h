#ifndef DISTANCE_CONSTRAINT_H
#define DISTANCE_CONSTRAINT_H

#include "Body.h"
#include "Constraint.h"

namespace Proton {

class DistanceConstraint final : public Constraint {
public:
  DistanceConstraint(
      Body* bodyA,
      Body* bodyB
  );

  void computePositionError(VectorXd& phi, int startRow) const override;
  void computeJacobian(MatrixXd& jacobian, int startRow) const override;
  void computeAccelerationCorrection(VectorXd& gamma, int startRow) const override;

  [[nodiscard]] Body* getBodyA() const { return m_bodyA; }
  [[nodiscard]] Body* getBodyB() const { return m_bodyB; }
  [[nodiscard]] double getDistance() const { return m_distance; }

private:
  Body* m_bodyA;
  Body* m_bodyB;
  double m_distance;

};
} // Proton

#endif // DISTANCE_CONSTRAINT_H
