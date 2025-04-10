#include "CylindricalJoint.h"

namespace Proton {
void CylindricalJoint::computePositionError(VectorXd &phi, int startRow) const {
  const auto& r1 = m_bodyA->getPosition();
  const auto& r2 = m_bodyB->getPosition();

  const auto& A1 = quaternionToRotationMatrix(m_bodyA->getOrientation());
  const auto& A2 = quaternionToRotationMatrix(m_bodyB->getOrientation());

  const auto& d = r2 + A2 * m_localPointB - r1 - A1 * m_localPointA;

  phi[startRow + 0] = (A1 * m_axisM1).eval().transpose() * d;
  phi[startRow + 1] = (A1 * m_axisN1).eval().transpose() * d;
  phi[startRow + 2] = (A2 * m_axisM2).eval().transpose() * (A1 * m_axisA).eval();
  phi[startRow + 3] = (A2 * m_axisN2).eval().transpose() * (A1 * m_axisA).eval();

}

void CylindricalJoint::computeJacobian(MatrixXd &jacobian, int startRow) const {
  const auto& r1 = m_bodyA->getPosition();
  const auto& r2 = m_bodyB->getPosition();
  const auto& A1 = quaternionToRotationMatrix(m_bodyA->getOrientation());
  const auto& A2 = quaternionToRotationMatrix(m_bodyB->getOrientation());
  const auto& d = r2 + A2 * m_localPointB - r1 - A1 * m_localPointA;

  jacobian.block<1,3>(startRow, m_bodyA->getIndex()*6).noalias()      = - 2.0 * d.transpose();
  jacobian.block<1,3>(startRow, m_bodyA->getIndex()*6 + 3).noalias()  =   2.0 * d.transpose() * A1 * skew(m_localPointA);
  jacobian.block<1,3>(startRow, m_bodyB->getIndex()*6).noalias()      =   2.0 * d.transpose();
  jacobian.block<1,3>(startRow, m_bodyB->getIndex()*6 + 3).noalias()  = - 2.0 * d.transpose() * A2 * skew(m_localPointB);

}

void CylindricalJoint::computeAccelerationCorrection(VectorXd &gamma, int startRow) const {


}
} // namespace Proton