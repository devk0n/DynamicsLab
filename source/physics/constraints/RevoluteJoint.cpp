#include "RevoluteJoint.h"

namespace Proton {

void RevoluteJoint::computePositionError(VectorXd &phi, int startRow) const {
  const auto& r1 = m_bodyA->getPosition();
  const auto& r2 = m_bodyB->getPosition();

  const auto& A1 = quaternionToRotationMatrix(m_bodyA->getOrientation());
  const auto& A2 = quaternionToRotationMatrix(m_bodyB->getOrientation());

  // Point constraint (3 rows)
  phi.segment<3>(startRow).noalias() = r1 + A1 * m_localPointA - r2 - A2 * m_localPointB;

  phi.segment<1>(startRow + 3).noalias() = (A1 * m_axisA).eval().transpose() * (A2 * m_axisM).eval();
  phi.segment<1>(startRow + 4).noalias() = (A1 * m_axisA).eval().transpose() * (A2 * m_axisN).eval();
}

void RevoluteJoint::computeJacobian(MatrixXd &jacobian, int startRow) const {
  const auto& A1 = quaternionToRotationMatrix(m_bodyA->getOrientation());
  const auto& A2 = quaternionToRotationMatrix(m_bodyB->getOrientation());

  // Point constraint Jacobian
  jacobian.block<3,3>(startRow, m_bodyA->getIndex() * 6).noalias()      =   Matrix3d::Identity();
  jacobian.block<3,3>(startRow, m_bodyA->getIndex() * 6 + 3).noalias()  = - A1 * skew(m_localPointA);
  jacobian.block<3,3>(startRow, m_bodyB->getIndex() * 6).noalias()      = - Matrix3d::Identity();
  jacobian.block<3,3>(startRow, m_bodyB->getIndex() * 6 + 3).noalias()  =   A2 * skew(m_localPointB);

  // Perpendicularity constraints Jacobian
  jacobian.block<1,3>(startRow + 3, m_bodyA->getIndex() * 6 + 3).noalias() = (A2 * m_axisM).transpose() * (A1 * skew(m_axisA));
  jacobian.block<1,3>(startRow + 3, m_bodyB->getIndex() * 6 + 3).noalias() = (A1 * m_axisA).transpose() * (A2 * skew(m_axisM));
  jacobian.block<1,3>(startRow + 4, m_bodyA->getIndex() * 6 + 3).noalias() = (A2 * m_axisN).transpose() * (A1 * skew(m_axisA));
  jacobian.block<1,3>(startRow + 4, m_bodyB->getIndex() * 6 + 3).noalias() = (A1 * m_axisA).transpose() * (A2 * skew(m_axisN));
}

void RevoluteJoint::computeAccelerationCorrection(VectorXd &gamma, int startRow) const {
  const auto& A1 = quaternionToRotationMatrix(m_bodyA->getOrientation());
  const auto& A2 = quaternionToRotationMatrix(m_bodyB->getOrientation());

  const auto& omega1 = m_bodyA->getAngularVelocity();
  const auto& omega2 = m_bodyB->getAngularVelocity();

  // World space quantities
  const auto& r1 = A1 * m_localPointA;
  const auto& r2 = A2 * m_localPointB;
  const auto& worldAxisA = A1 * m_axisA;
  const auto& worldAxisM = A2 * m_axisM;
  const auto& worldAxisN = A2 * m_axisN;

  // Point constraint acceleration (3 rows)
  gamma.segment<3>(startRow).noalias() =
      omega1.cross(omega1.cross(r1)) - omega2.cross(omega2.cross(r2));

  // Perpendicularity constraints acceleration (2 rows)
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
