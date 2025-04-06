#include "UniversalJoint.h"

#include <utility>

namespace Proton {
void UniversalJoint::computePositionError(VectorXd& phi, int startRow) const {
  const auto& r1 = m_bodyA->getPosition();
  const auto& r2 = m_bodyB->getPosition();

  const auto& A1 = quaternionToRotationMatrix(m_bodyA->getOrientation());
  const auto& A2 = quaternionToRotationMatrix(m_bodyB->getOrientation());

  phi.segment<3>(startRow).noalias() = r1 + A1 * m_localPointA - r2 - A2 * m_localPointB;
  phi.segment<1>(startRow + 3).noalias() = (A1 * m_axisA).eval().transpose() * (A2 * m_axisB).eval();
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
  jacobian.block<1,3>(startRow + 3, m_bodyA->getIndex() * 6 + 3).noalias() = (A2 * m_axisB).transpose() * (A1 * skew(m_axisA));
  jacobian.block<1,3>(startRow + 3, m_bodyB->getIndex() * 6 + 3).noalias() = (A1 * m_axisA).transpose() * (A2 * skew(m_axisB));

}

void UniversalJoint::computeAccelerationCorrection(VectorXd& gamma, int startRow) const {
  const auto& A1 = quaternionToRotationMatrix(m_bodyA->getOrientation());
  const auto& A2 = quaternionToRotationMatrix(m_bodyB->getOrientation());

  const auto& omega1 = m_bodyA->getAngularVelocity();
  const auto& omega2 = m_bodyB->getAngularVelocity();

  const auto& result = A1 * (skew(omega1) * skew(m_localPointA)).eval() * omega1 - A2 * (skew(omega2) * skew(m_localPointB)).eval() * omega2;
  gamma.segment<3>(startRow).noalias() = result;

  const auto& a1 = A1 * m_axisA;
  const auto& a2 = A2 * m_axisB;

  double cTerm = 0.0;

  //  a1 dot [ ω2 × ( ω2 × a2 ) ]
  cTerm += a1.dot(omega2.cross(omega2.cross(a2)));

  //  + 2 ( ω1×a1 ) dot ( ω2×a2 )
  cTerm += 2.0 * (omega1.cross(a1)).dot(omega2.cross(a2));

  //  + [ ω1 × ( ω1 × a1 ) ] dot a2
  cTerm += (omega1.cross(omega1.cross(a1))).dot(a2);

  // gamma(4th constraint) = - (that expression)
  gamma[startRow + 3] = -cTerm;
}
} // Proton