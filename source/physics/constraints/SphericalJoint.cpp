#include "SphericalJoint.h"

namespace Proton {

void SphericalJoint::computePositionError(VectorXd &phi, int startRow) const {
  const auto &r1 = m_bodyA->getPosition();
  const auto &r2 = m_bodyB->getPosition();

  const auto &A1 = quaternionToRotationMatrix(m_bodyA->getOrientation());
  const auto &A2 = quaternionToRotationMatrix(m_bodyB->getOrientation());

  const auto &d = r2 + A2 * m_localPointB - r1 - A1 * m_localPointA;

  phi(startRow) = d.transpose() * d - (m_distance * m_distance);
}

void SphericalJoint::computeJacobian(MatrixXd &jacobian, int startRow) const {
  const auto &r1 = m_bodyA->getPosition();
  const auto &r2 = m_bodyB->getPosition();
  const auto &A1 = quaternionToRotationMatrix(m_bodyA->getOrientation());
  const auto &A2 = quaternionToRotationMatrix(m_bodyB->getOrientation());
  const auto &d = r2 + A2 * m_localPointB - r1 - A1 * m_localPointA;

  jacobian.block<1,3>(startRow, m_bodyA->getIndex()*6).noalias()      = - 2.0 * d.transpose();
  jacobian.block<1,3>(startRow, m_bodyA->getIndex()*6 + 3).noalias()  =   2.0 * d.transpose() * A1 * skew(m_localPointA);
  jacobian.block<1,3>(startRow, m_bodyB->getIndex()*6).noalias()      =   2.0 * d.transpose();
  jacobian.block<1,3>(startRow, m_bodyB->getIndex()*6 + 3).noalias()  = - 2.0 * d.transpose() * A2 * skew(m_localPointB);
}

void SphericalJoint::computeAccelerationCorrection(VectorXd &gamma, int startRow) const {
  const auto &r1 = m_bodyA->getPosition();
  const auto &r2 = m_bodyB->getPosition();

  const auto &A1 = quaternionToRotationMatrix(m_bodyA->getOrientation());
  const auto &A2 = quaternionToRotationMatrix(m_bodyB->getOrientation());

  const auto &omega1 = m_bodyA->getAngularVelocity();
  const auto &omega2 = m_bodyB->getAngularVelocity();
  const auto &r1d = m_bodyA->getLinearVelocity();
  const auto &r2d = m_bodyB->getLinearVelocity();

  const auto &d = r2 + A2 * m_localPointB - r1 - A1 * m_localPointA;
  const auto &v = r2d + A2 * skew(omega2) * m_localPointB - r1d - A1 * skew(omega1) * m_localPointA;

  const double &vtv = v.transpose() * v;

  const auto &skewW2 = skew(omega2);
  const auto &skewW1 = skew(omega1);

  const auto &termB = A2 * skewW2 * skewW2 * m_localPointB;
  const auto &termA = A1 * skewW1 * skewW1 * m_localPointA;
  const auto &par = termB - termA;

  const double &dot = d.transpose() * par;
  const double &result = -2.0 * vtv - 2.0 * dot;

  gamma[startRow] = result;
}

void SphericalJoint::computeDistance() {
  // Compute each anchor’s world‑space position
  const auto &A1 = quaternionToRotationMatrix(m_bodyA->getOrientation());
  const auto &A2 = quaternionToRotationMatrix(m_bodyB->getOrientation());
  const auto &world1 = m_bodyA->getPosition() + A1 * m_localPointA;
  const auto &world2 = m_bodyB->getPosition() + A2 * m_localPointB;

  // Initialize the resting distance automatically
  m_distance = (world2 - world1).norm();
}
} // Proton