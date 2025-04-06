#ifndef DISTANCE_CONSTRAINT_H
#define DISTANCE_CONSTRAINT_H

#include "Body.h"
#include "Constraint.h"

namespace Proton {
class DistanceConstraint final : public Constraint {
public:
  explicit DistanceConstraint() : Constraint(1) {}

  void computePositionError(VectorXd& phi, int startRow) const override;
  void computeJacobian(MatrixXd& jacobian, int startRow) const override;
  void computeAccelerationCorrection(VectorXd& gamma, int startRow) const override;

  void computeDistance();

  [[nodiscard]] Body* getBodyA() const { return m_bodyA; }
  [[nodiscard]] Body* getBodyB() const { return m_bodyB; }
  [[nodiscard]] const Vector3d& getLocalPointA() const { return m_localPointA; }
  [[nodiscard]] const Vector3d& getLocalPointB() const { return m_localPointB; }
  [[nodiscard]] const double& getDistance() const { return m_distance; }

  void setBodyA(Body* bodyA) { m_bodyA = bodyA; }
  void setBodyB(Body* bodyB) { m_bodyB = bodyB; }
  void setLocalPointA(const Vector3d& localA) { m_localPointA = localA; }
  void setLocalPointB(const Vector3d& localB) { m_localPointB = localB; }
  void setDistance(const double& distance) { m_distance = distance; }

private:
  Body* m_bodyA{nullptr};
  Body* m_bodyB{nullptr};
  Vector3d m_localPointA{0, 0, 0};
  Vector3d m_localPointB{0, 0, 0};
  double m_distance{1};

};
} // Proton
#endif // DISTANCE_CONSTRAINT_H
