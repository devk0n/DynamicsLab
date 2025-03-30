#ifndef SPHERICAL_JOINT_H
#define SPHERICAL_JOINT_H

#include "Body.h"
#include "Constraint.h"

namespace Proton {
class SphericalJoint final : public Constraint {
public:
  SphericalJoint();

  SphericalJoint(
      Body* bodyA, Vector3d localPointA,
      Body* bodyB, Vector3d localPointB
  );

  SphericalJoint(
      Body* bodyA, Vector3d localPointA,
      Body* bodyB, Vector3d localPointB,
      double distance
  );

  void computePositionError(VectorXd &phi, int startRow) const override;
  void computeJacobian(MatrixXd &jacobian, int startRow) const override;
  void computeAccelerationCorrection(VectorXd &gamma, int startRow) const override;
  void computeDistance();

  [[nodiscard]] Body* getBodyA() const { return m_bodyA; }
  [[nodiscard]] Body* getBodyB() const { return m_bodyB; }
  [[nodiscard]] Vector3d getLocalPointA() const { return m_localPointA; }
  [[nodiscard]] Vector3d getLocalPointB() const { return m_localPointB; }
  [[nodiscard]] double getDistance() const { return m_distance; }

  void setBodyA(Body* bodyA) { m_bodyA = bodyA; }
  void setBodyB(Body* bodyB) { m_bodyB = bodyB; }
  void setLocalPointA(Vector3d localA) { m_localPointA = std::move(localA); }
  void setLocalPointB(Vector3d localB) { m_localPointB = std::move(localB); }
  void setDistance(double distance) { m_distance = distance; }

private:
  Body* m_bodyA;
  Body* m_bodyB;
  Vector3d m_localPointA{0, 0, 0};
  Vector3d m_localPointB{0, 0, 0};
  double m_distance{};
};
} // Proton

#endif // SPHERICAL_JOINT_H
