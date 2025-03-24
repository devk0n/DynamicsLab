#include "SphericalJoint.h"

namespace Proton {

SphericalJoint::SphericalJoint(
    Body *body1,
    const Vector3d &local1,
    Body *body2,
    const Vector3d &local2)
    : Constraint(1),
      m_body1(body1),
      m_body2(body2),
      m_local1(local1),
      m_local2(local2) {
  m_distance = 0.0;
}

void SphericalJoint::computePositionError(VectorXd &phi, const int startRow) const {

  auto A1 = matrixG(m_body1->getOrientation()) * matrixL(m_body1->getOrientation()).transpose();
  auto A2 = matrixG(m_body2->getOrientation()) * matrixL(m_body2->getOrientation()).transpose();

  auto d = m_body2->getPosition() + A2 * m_local2 - (m_body1->getPosition() + A1 * m_local1);

  // Constraint equation
  auto result = (d.transpose() * d) - (m_distance * m_distance);

  phi[startRow] = result;
}

void SphericalJoint::computeJacobian(MatrixXd &jacobian, const int startRow) const {
  auto A1 = matrixG(m_body1->getOrientation()) * matrixL(m_body1->getOrientation()).transpose();
  auto A2 = matrixG(m_body2->getOrientation()) * matrixL(m_body2->getOrientation()).transpose();

  // Relative position
  auto d = m_body2->getPosition() + A2 * m_local2 - (m_body1->getPosition() + A1 * m_local1);

  // Jacobian matrix
  jacobian.block<1, 3>(startRow, m_body1->getIndex() * 6)     = - 2 * d.transpose();
  jacobian.block<1, 3>(startRow, m_body1->getIndex() * 6 + 3) =   2 * d.transpose() * A1 * skew(m_local1);
  jacobian.block<1, 3>(startRow, m_body2->getIndex() * 6)     =   2 * d.transpose();
  jacobian.block<1, 3>(startRow, m_body2->getIndex() * 6 + 3) = - 2 * d.transpose() * A2 * skew(m_local2);
}

void SphericalJoint::computeAccelerationCorrection(VectorXd &gamma, const int startRow) const {
  auto A1 = matrixG(m_body1->getOrientation()) * matrixL(m_body1->getOrientation()).transpose();
  auto A2 = matrixG(m_body2->getOrientation()) * matrixL(m_body2->getOrientation()).transpose();

  auto omega1 = m_body1->getAngularVelocity();
  auto omega2 = m_body2->getAngularVelocity();

  auto v = m_body2->getLinearVelocity() + A2 * skew(m_body2->getAngularVelocity()) * m_local2 -
              (m_body1->getLinearVelocity() + A1 * skew(m_body1->getAngularVelocity()) * m_local1);

  auto o = - 2 * (v.transpose() * v);

  auto in = A2 * skew(omega2) * skew(omega2) * m_local2 + A1 * skew(omega1) * skew(omega1) * m_local1;

  auto result = o - (2 * v.transpose() * in);

  gamma[startRow] = result.value();
}

}


