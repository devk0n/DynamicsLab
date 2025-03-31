#ifndef REVOLUTE_JOINT_H
#define REVOLUTE_JOINT_H

#include "Body.h"
#include "Constraint.h"

namespace Proton {

class RevoluteJoint final : public Constraint {
public:
  RevoluteJoint(
      Body* bodyA, Vector3d localPointA, Vector3d axisA,
      Body* bodyB, Vector3d localPointB, Vector3d axisB
  );

  RevoluteJoint() : Constraint(5), m_bodyA(nullptr), m_bodyB(nullptr) {}

  void computePositionError(VectorXd &phi, int startRow) const override;
  void computeJacobian(MatrixXd &jacobian, int startRow) const override;
  void computeAccelerationCorrection(VectorXd &gamma, int startRow) const override;

  void calculateConstraintAxes();

  [[nodiscard]] Body* getBodyA() const { return m_bodyA; }
  [[nodiscard]] Body* getBodyB() const { return m_bodyB; }
  [[nodiscard]] Vector3d getLocalPointA() const { return m_localPointA; }
  [[nodiscard]] Vector3d getLocalPointB() const { return m_localPointB; }
  [[nodiscard]] Vector3d getAxisA() const { return m_axisA; }
  [[nodiscard]] Vector3d getAxisB() const { return m_axisB; }
  [[nodiscard]] Vector3d getAxisM() const { return m_axisM; }
  [[nodiscard]] Vector3d getAxisN() const { return m_axisN; }

  void setBodyA(Body* bodyA) { m_bodyA = bodyA; }
  void setBodyB(Body* bodyB) { m_bodyB = bodyB; }
  void setLocalPointA(Vector3d localA) { m_localPointA = std::move(localA); }
  void setLocalPointB(Vector3d localB) { m_localPointB = std::move(localB); }
  void setAxisA(Vector3d axisA) { m_axisA = std::move(axisA); }
  void setAxisB(Vector3d axisB) { m_axisB = std::move(axisB); }
  void setAxisM(Vector3d axisM) { m_axisM = std::move(axisM); }
  void setAxisN(Vector3d axisN) { m_axisN = std::move(axisN); }

private:
  Body* m_bodyA;
  Body* m_bodyB;
  Vector3d m_localPointA;
  Vector3d m_localPointB;
  Vector3d m_axisA;
  Vector3d m_axisB;
  Vector3d m_axisM;
  Vector3d m_axisN;
};

} // Proton

#endif // REVOLUTE_JOINT_H
