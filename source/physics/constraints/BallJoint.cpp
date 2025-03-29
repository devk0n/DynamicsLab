#include "BallJoint.h"

#include <utility>

namespace Proton {

BallJoint::BallJoint(
    Body *bodyA,
    Vector3d localPointA,
    Body *bodyB,
    Vector3d localPointB)
    : Constraint(3),
      m_bodyA(bodyA),
      m_bodyB(bodyB),
      m_localPointA(std::move(localPointA)),
      m_localPointB(std::move(localPointB)) {}

void BallJoint::computePositionError(VectorXd &phi, const int startRow) const {
  auto rA = m_bodyA->getPosition();
  auto rB = m_bodyB->getPosition();

  auto AA = quaternionToRotationMatrix(m_bodyA->getOrientation());
  auto AB = quaternionToRotationMatrix(m_bodyB->getOrientation());

  phi.segment<3>(startRow) = rA + AA * m_localPointA - rB - AB * m_localPointB;
}

void BallJoint::computeJacobian(MatrixXd &jacobian, const int startRow) const {

  auto AA = quaternionToRotationMatrix(m_bodyA->getOrientation());
  auto AB = quaternionToRotationMatrix(m_bodyB->getOrientation());

  // Jacobian matrix
  jacobian.block<3, 3>(startRow, m_bodyA->getIndex() * 6)     =   Matrix3d::Identity(); // BodyA linear
  jacobian.block<3, 3>(startRow, m_bodyA->getIndex() * 6 + 3) = - AA * skew(m_localPointA);    // BodyA angular
  jacobian.block<3, 3>(startRow, m_bodyB->getIndex() * 6)     = - Matrix3d::Identity(); // BodyB linear
  jacobian.block<3, 3>(startRow, m_bodyB->getIndex() * 6 + 3) =   AB * skew(m_localPointB);    // BodyB angular
}

void BallJoint::computeAccelerationCorrection(VectorXd &gamma, const int startRow) const {
  auto AA = quaternionToRotationMatrix(m_bodyA->getOrientation());
  auto AB = quaternionToRotationMatrix(m_bodyB->getOrientation());

  auto omegaA = m_bodyA->getAngularVelocity();
  auto omegaB = m_bodyB->getAngularVelocity();

  auto result = AA * (skew(omegaA) * skew(m_localPointA)).eval() * omegaA - AB * (skew(omegaB) * skew(m_localPointB)).eval() * omegaB;
  gamma.segment<3>(startRow) = result;
}

}


