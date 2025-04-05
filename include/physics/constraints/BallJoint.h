#ifndef BALL_JOINT_H
#define BALL_JOINT_H

#include <utility>

#include "Body.h"
#include "Constraint.h"

namespace Proton {

class BallJoint final : public Constraint {
public:
  BallJoint(
      Body* bodyA, Vector3d localPointA,
      Body* bodyB, Vector3d localPointB
  );

  BallJoint() : Constraint(3), m_bodyA(nullptr), m_bodyB(nullptr) {}

  void computePositionError(VectorXd& phi, int startRow) const override;
  void computeJacobian(MatrixXd& jacobian, int startRow) const override;
  void computeAccelerationCorrection(VectorXd& gamma, int startRow) const override;

  [[nodiscard]] Body* getBodyA() const { return m_bodyA; }
  [[nodiscard]] Body* getBodyB() const { return m_bodyB; }
  [[nodiscard]] Vector3d getLocalPointA() const { return m_localPointA; }
  [[nodiscard]] Vector3d getLocalPointB() const { return m_localPointB; }

  void setBodyA(Body* bodyA) { m_bodyA = bodyA; }
  void setBodyB(Body* bodyB) { m_bodyB = bodyB; }
  void setLocalPointA(Vector3d localA) { m_localPointA = std::move(localA); }
  void setLocalPointB(Vector3d localB) { m_localPointB = std::move(localB); }

private:
  Body* m_bodyA{nullptr};
  Body* m_bodyB{nullptr};
  Vector3d m_localPointA{0, 0, 0};
  Vector3d m_localPointB{0, 0, 0};
};
} // Proton

#endif // BALL_JOINT_H
