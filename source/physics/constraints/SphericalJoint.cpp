#include "SphericalJoint.h"

namespace Proton {
SphericalJoint::SphericalJoint(
    Body *body1, Vector3d local1,
    Body *body2, Vector3d local2)
    : Constraint(1),
      m_body1(body1),
      m_body2(body2),
      m_local1(std::move(local1)),
      m_local2(std::move(local2)) {
  m_distance = 9.0;
}

void SphericalJoint::computePositionError(VectorXd &phi, int startRow) const {
  auto r1 = m_body1->getPosition();
  auto r2 = m_body2->getPosition();

  auto A1 = quaternionToRotationMatrix(m_body1->getOrientation());
  auto A2 = quaternionToRotationMatrix(m_body2->getOrientation());

  auto d = r2 + (A2 * m_local2).eval() - (r1 + (A1 * m_local1).eval());

  phi[startRow] = (d.transpose() * d) - (m_distance * m_distance);
}

void SphericalJoint::computeJacobian(MatrixXd &jacobian, int startRow) const {

  auto r1 = m_body1->getPosition();
  auto r2 = m_body2->getPosition();

  auto A1 = quaternionToRotationMatrix(m_body1->getOrientation());
  auto A2 = quaternionToRotationMatrix(m_body2->getOrientation());

  auto d = r2 + (A2 * m_local2).eval() - (r1 + (A1 * m_local1).eval());

  // Jacobian matrix
  jacobian.block<1, 3>(startRow, m_body1->getIndex() * 6)     =   2 * d.transpose();
  jacobian.block<1, 3>(startRow, m_body1->getIndex() * 6 + 3) = - 2 * d.transpose() * A1 * skew(m_local1);
  jacobian.block<1, 3>(startRow, m_body2->getIndex() * 6)     =   2 * d.transpose();
  jacobian.block<1, 3>(startRow, m_body2->getIndex() * 6 + 3) = - 2 * d.transpose() * A2 * skew(m_local2);
}

void SphericalJoint::computeAccelerationCorrection(VectorXd &gamma, int startRow) const {



  gamma(startRow) = 0.0;

}
} // Proton