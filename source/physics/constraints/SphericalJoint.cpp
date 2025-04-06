#include "SphericalJoint.h"

namespace Proton {

void SphericalJoint::computePositionError(VectorXd& phi, const int startRow) const {
  const auto& rA = m_bodyA->getPosition();
  const auto& rB = m_bodyB->getPosition();

  const auto& AA = quaternionToRotationMatrix(m_bodyA->getOrientation());
  const auto& AB = quaternionToRotationMatrix(m_bodyB->getOrientation());

  phi.segment<3>(startRow).noalias() = rA + AA * m_localPointA - rB - AB * m_localPointB;
}

void SphericalJoint::computeJacobian(MatrixXd& jacobian, const int startRow) const {

  const auto& AA = quaternionToRotationMatrix(m_bodyA->getOrientation());
  const auto& AB = quaternionToRotationMatrix(m_bodyB->getOrientation());

  // Jacobian matrix
  jacobian.block<3,3>(startRow, m_bodyA->getIndex() * 6).noalias()     =   Matrix3d::Identity();     // BodyA linear
  jacobian.block<3,3>(startRow, m_bodyA->getIndex() * 6 + 3).noalias() = - AA * skew(m_localPointA); // BodyA angular
  jacobian.block<3,3>(startRow, m_bodyB->getIndex() * 6).noalias()     = - Matrix3d::Identity();     // BodyB linear
  jacobian.block<3,3>(startRow, m_bodyB->getIndex() * 6 + 3).noalias() =   AB * skew(m_localPointB); // BodyB angular
}

void SphericalJoint::computeAccelerationCorrection(VectorXd& gamma, const int startRow) const {
  const auto& A1 = quaternionToRotationMatrix(m_bodyA->getOrientation());
  const auto& A2 = quaternionToRotationMatrix(m_bodyB->getOrientation());

  const auto& omegaA = m_bodyA->getAngularVelocity();
  const auto& omegaB = m_bodyB->getAngularVelocity();

  const auto& result = A1 * (skew(omegaA) * skew(m_localPointA)).eval() * omegaA - A2 * (skew(omegaB) * skew(m_localPointB)).eval() * omegaB;
  gamma.segment<3>(startRow).noalias() = result;
}

} // namespace Proton
