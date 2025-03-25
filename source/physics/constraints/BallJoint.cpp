#include "BallJoint.h"

#include <utility>

namespace Proton {

BallJoint::BallJoint(
    Body *body1,
    Vector3d local1,
    Body *body2,
    Vector3d local2)
    : Constraint(3),
      m_body1(body1),
      m_body2(body2),
      m_local1(std::move(local1)),
      m_local2(std::move(local2)) {}

void BallJoint::computePositionError(VectorXd &phi, const int startRow) const {
  auto r1 = m_body1->getPosition();
  auto r2 = m_body2->getPosition();

  auto A1 = quaternionToRotationMatrix(m_body1->getOrientation());
  auto A2 = quaternionToRotationMatrix(m_body2->getOrientation());

  phi.segment<3>(startRow) = r1 + (A1 * m_local1).eval() - (r2 + (A2 * m_local2).eval());
}

void BallJoint::computeJacobian(MatrixXd &jacobian, const int startRow) const {

  auto A1 = quaternionToRotationMatrix(m_body1->getOrientation());
  auto A2 = quaternionToRotationMatrix(m_body2->getOrientation());

  // Jacobian matrix
  jacobian.block<3, 3>(startRow, m_body1->getIndex() * 6)     =   Matrix3d::Identity(); // Body1 linear
  jacobian.block<3, 3>(startRow, m_body1->getIndex() * 6 + 3) =   A1 * skew(m_local1);    // Body1 angular
  jacobian.block<3, 3>(startRow, m_body2->getIndex() * 6)     = - Matrix3d::Identity(); // Body2 linear
  jacobian.block<3, 3>(startRow, m_body2->getIndex() * 6 + 3) = - A2 * skew(m_local2);    // Body2 angular
}

void BallJoint::computeAccelerationCorrection(VectorXd &gamma, const int startRow) const {
  auto A1 = quaternionToRotationMatrix(m_body1->getOrientation());
  auto A2 = quaternionToRotationMatrix(m_body2->getOrientation());

  auto worldLocal1 = A1 * skew(m_local1);
  auto worldLocal2 = A2 * skew(m_local2);

  auto worldLocal1t = worldLocal1 * skew(m_local1);
  auto worldLocal2t = worldLocal2 * skew(m_local2);

  auto omega1 = m_body1->getAngularVelocity();
  auto omega2 = m_body2->getAngularVelocity();

  auto result = worldLocal1t - worldLocal2t;

  gamma.segment<3>(startRow) = Vector3d::Zero();
}

}


