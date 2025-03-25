#include "RevoluteJoint.h"

namespace Proton {
RevoluteJoint::RevoluteJoint(
    Body *body1, const Vector3d &local1,
    Body *body2, const Vector3d &local2)
    : Constraint(5),
      m_body1(body1),
      m_body2(body2),
      m_local1(local1),
      m_local2(local2) {}

void RevoluteJoint::computePositionError(VectorXd &phi, int startRow) const {

  auto r1 = m_body1->getPosition();
  auto r2 = m_body2->getPosition();

  auto A1 = quaternionToRotationMatrix(m_body1->getOrientation());
  auto A2 = quaternionToRotationMatrix(m_body2->getOrientation());

  phi.segment<3>(startRow) = r2 + A2 * m_local2 - r1 - A1 * m_local1;
}

void RevoluteJoint::computeJacobian(MatrixXd &jacobian, int startRow) const {

  const auto r1 = m_body1->getPosition();
  const auto r2 = m_body2->getPosition();

  const auto A1 = quaternionToRotationMatrix(m_body1->getOrientation());
  const auto A2 = quaternionToRotationMatrix(m_body2->getOrientation());

  const auto d = r2 + A2 * m_local2 - r1 - A1 * m_local1;

  jacobian.block<1,3>(startRow, m_body1->getIndex()*6)      = - 2.0 * d.transpose();
  jacobian.block<1,3>(startRow, m_body1->getIndex()*6 + 3)  =   2.0 * d.transpose() * A1 * skew(m_local1);
  jacobian.block<1,3>(startRow, m_body2->getIndex()*6)      =   2.0 * d.transpose();
  jacobian.block<1,3>(startRow, m_body2->getIndex()*6 + 3)  = - 2.0 * d.transpose() * A2 * skew(m_local2);

}

void RevoluteJoint::computeAccelerationCorrection(VectorXd &gamma, int startRow) const {

  const auto r1 = m_body1->getPosition();
  const auto r2 = m_body2->getPosition();

  const auto A1 = quaternionToRotationMatrix(m_body1->getOrientation());
  const auto A2 = quaternionToRotationMatrix(m_body2->getOrientation());

  const auto omega1 = m_body1->getAngularVelocity();
  const auto omega2 = m_body2->getAngularVelocity();

  const auto r1d = m_body1->getLinearVelocity();
  const auto r2d = m_body2->getLinearVelocity();

  const auto d = r2 + A2 * m_local2 - r1 - A1 * m_local1;
  auto v = r2d + A2 * skew(omega1) * m_local2 - r1d - A1 * skew(omega2) * m_local1;

  auto vtv = (v.transpose() * v);
  auto par = A2 * skew(omega2) * skew(omega2) * m_local2 - (A1 * skew(omega1) * skew(omega1) * m_local1);

  auto result = - 2 * vtv - 2 * (d.transpose() * par).eval();

  gamma.segment<1>(startRow) = result;

}
} // Proton
