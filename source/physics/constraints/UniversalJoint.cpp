#include "UniversalJoint.h"

namespace Proton {

void UniversalJoint::computePositionError(VectorXd& phi, int startRow) const {
  const auto& rA = m_bodyA->getPosition();
  const auto& rB = m_bodyB->getPosition();

  const auto& AA = quaternionToRotationMatrix(m_bodyA->getOrientation());
  const auto& AB = quaternionToRotationMatrix(m_bodyB->getOrientation());

  phi.segment<3>(startRow).noalias() = rA + AA * m_localPointA - rB - AB * m_localPointB;
  phi.segment<1>(startRow + 3).noalias() = (AA * m_axisA).eval().transpose() * (AB * m_axisB).eval();
}

void UniversalJoint::computeJacobian(MatrixXd& jacobian, int startRow) const {
  const auto& A1 = quaternionToRotationMatrix(m_bodyA->getOrientation());
  const auto& A2 = quaternionToRotationMatrix(m_bodyB->getOrientation());

  // Jacobian matrix Spherical
  jacobian.block<3,3>(startRow, m_bodyA->getIndex() * 6).noalias()     =   Matrix3d::Identity();
  jacobian.block<3,3>(startRow, m_bodyA->getIndex() * 6 + 3).noalias() = - A1 * skew(m_localPointA);
  jacobian.block<3,3>(startRow, m_bodyB->getIndex() * 6).noalias()     = - Matrix3d::Identity();
  jacobian.block<3,3>(startRow, m_bodyB->getIndex() * 6 + 3).noalias() =   A2 * skew(m_localPointB);

  // Jacobian matrix Universal
  jacobian.block<1,3>(startRow + 3, m_bodyA->getIndex() * 6 + 3).noalias() = -(A2 * m_axisB).transpose() * (A1 * skew(m_axisA));
  jacobian.block<1,3>(startRow + 3, m_bodyB->getIndex() * 6 + 3).noalias() = -(A1 * m_axisA).transpose() * (A2 * skew(m_axisB));

}

void UniversalJoint::computeAccelerationCorrection(VectorXd& gamma, int startRow) const {
  const auto& RA = quaternionToRotationMatrix(m_bodyA->getOrientation());
  const auto& RB = quaternionToRotationMatrix(m_bodyB->getOrientation());

  const auto& omegaA = m_bodyA->getAngularVelocity();
  const auto& omegaB = m_bodyB->getAngularVelocity();

  const auto& termA = RA * (skew(omegaA) * skew(m_localPointA) * omegaA);
  const auto& termB = RB * (skew(omegaB) * skew(m_localPointB) * omegaB);

  gamma.segment<3>(startRow).noalias() = termA - termB;

  const auto& a1 = RA * m_axisA;
  const auto& a2 = RB * m_axisB;

  double cTerm = 0.0;

  cTerm += a1.dot(omegaB.cross(omegaB.cross(a2)));
  cTerm += 2.0 * (omegaA.cross(a1)).dot(omegaB.cross(a2));
  cTerm += (omegaA.cross(omegaA.cross(a1))).dot(a2);

  gamma[startRow + 3] = -cTerm;
}
} // Proton