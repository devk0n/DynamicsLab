#include "RevoluteJoint.h"

#include <utility>

namespace Proton {
RevoluteJoint::RevoluteJoint(
    Body* bodyA, Vector3d localPointA, Vector3d axisA,
    Body* bodyB, Vector3d localPointB, Vector3d axisB)
      : Constraint(5),
        m_bodyA(bodyA),
        m_bodyB(bodyB),
        m_localPointA(std::move(localPointA)),
        m_localPointB(std::move(localPointB)),
        m_axisA(std::move(axisA)),
        m_axisB(std::move(axisB)) {
  m_axisM = m_axisA.cross(Vector3d(1, 0, 0)).normalized();  // Perpendicular to axisA
  m_axisN = m_axisA.cross(m_axisM).normalized();             // Perpendicular to both axisA and axisM
}

void RevoluteJoint::computePositionError(VectorXd &phi, int startRow) const {
  auto r1 = m_bodyA->getPosition();
  auto r2 = m_bodyB->getPosition();

  auto A1 = quaternionToRotationMatrix(m_bodyA->getOrientation());
  auto A2 = quaternionToRotationMatrix(m_bodyB->getOrientation());

  // Point constraint (3 rows)
  phi.segment<3>(startRow) = r1 + A1 * m_localPointA - r2 - A2 * m_localPointB;

  phi.segment<1>(startRow + 3) = (A1 * m_axisA).eval().transpose() * (A2 * m_axisM).eval();
  phi.segment<1>(startRow + 4) = (A1 * m_axisA).eval().transpose() * (A2 * m_axisN).eval();
}

void RevoluteJoint::computeJacobian(MatrixXd &jacobian, int startRow) const {
  const auto A1 = quaternionToRotationMatrix(m_bodyA->getOrientation());
  const auto A2 = quaternionToRotationMatrix(m_bodyB->getOrientation());

  // Point constraint Jacobian
  jacobian.block<3,3>(startRow, m_bodyA->getIndex() * 6)      =   Matrix3d::Identity();
  jacobian.block<3,3>(startRow, m_bodyA->getIndex() * 6 + 3)  = - A1 * skew(m_localPointA);
  jacobian.block<3,3>(startRow, m_bodyB->getIndex() * 6)      = - Matrix3d::Identity();
  jacobian.block<3,3>(startRow, m_bodyB->getIndex() * 6 + 3)  =   A2 * skew(m_localPointB);

  // Perpendicularity constraints Jacobian
  jacobian.block<1,3>(startRow + 3, m_bodyA->getIndex() * 6 + 3) = (A2 * m_axisM).transpose() * (A1 * skew(m_axisA));
  jacobian.block<1,3>(startRow + 3, m_bodyB->getIndex() * 6 + 3) = (A1 * m_axisA).transpose() * (A2 * skew(m_axisM));
  jacobian.block<1,3>(startRow + 4, m_bodyA->getIndex() * 6 + 3) = (A2 * m_axisN).transpose() * (A1 * skew(m_axisA));
  jacobian.block<1,3>(startRow + 4, m_bodyB->getIndex() * 6 + 3) = (A1 * m_axisA).transpose() * (A2 * skew(m_axisN));
}

void RevoluteJoint::computeAccelerationCorrection(VectorXd &gamma, int startRow) const {
  const auto A1 = quaternionToRotationMatrix(m_bodyA->getOrientation());
  const auto A2 = quaternionToRotationMatrix(m_bodyB->getOrientation());

  const Vector3d omega1 = m_bodyA->getAngularVelocity();
  const Vector3d omega2 = m_bodyB->getAngularVelocity();

  // World space quantities
  const Vector3d r1 = A1 * m_localPointA;
  const Vector3d r2 = A2 * m_localPointB;
  const Vector3d worldAxisA = A1 * m_axisA;
  const Vector3d worldAxisM = A2 * m_axisM;
  const Vector3d worldAxisN = A2 * m_axisN;

  // Point constraint acceleration (3 rows)
  gamma.segment<3>(startRow) =
      omega1.cross(omega1.cross(r1)) - omega2.cross(omega2.cross(r2));

  // Perpendicularity constraints acceleration (2 rows)
  // γ = -2(ω₁×a)ᵀ(ω₂×m) - aᵀ(ω₂×(ω₂×m)) - (ω₁×(ω₁×a))ᵀm
  gamma[startRow + 3] =
      -2.0 * omega1.cross(worldAxisA).dot(omega2.cross(worldAxisM))
      - worldAxisA.dot(omega2.cross(omega2.cross(worldAxisM)))
      - omega1.cross(omega1.cross(worldAxisA)).dot(worldAxisM);

  gamma[startRow + 4] =
      -2.0 * omega1.cross(worldAxisA).dot(omega2.cross(worldAxisN))
      - worldAxisA.dot(omega2.cross(omega2.cross(worldAxisN)))
      - omega1.cross(omega1.cross(worldAxisA)).dot(worldAxisN);
}
} // Proton
