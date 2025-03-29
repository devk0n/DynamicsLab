#include "SphericalJoint.h"

namespace Proton {
SphericalJoint::SphericalJoint(
    Body *bodyA, Vector3d local1,
    Body *bodyB, Vector3d local2)
    : Constraint(1),
      m_bodyA(bodyA),
      m_bodyB(bodyB),
      m_localPointA(std::move(local1)),
      m_localPointB(std::move(local2)) {
  // Compute each anchor’s world‑space position
  const auto A1 = quaternionToRotationMatrix(m_bodyA->getOrientation());
  const auto A2 = quaternionToRotationMatrix(m_bodyB->getOrientation());
  const Vector3d world1 = m_bodyA->getPosition() + A1 * m_localPointA;
  const Vector3d world2 = m_bodyB->getPosition() + A2 * m_localPointB;

  // Initialize the resting distance automatically
  m_distance = (world2 - world1).norm();
}

SphericalJoint::SphericalJoint(
    Body *bodyA, Vector3d local1,
    Body *bodyB, Vector3d local2,
    const double distance)
    : Constraint(1),
      m_bodyA(bodyA),
      m_bodyB(bodyB),
      m_localPointA(std::move(local1)),
      m_localPointB(std::move(local2)),
      m_distance(distance) {}

void SphericalJoint::computePositionError(VectorXd &phi, int startRow) const {
  auto r1 = m_bodyA->getPosition();
  auto r2 = m_bodyB->getPosition();

  auto A1 = quaternionToRotationMatrix(m_bodyA->getOrientation());
  auto A2 = quaternionToRotationMatrix(m_bodyB->getOrientation());

  const auto d = r2 + A2 * m_localPointB - r1 - A1 * m_localPointA;

  phi(startRow) = (d.transpose() * d) - (m_distance * m_distance);
}

void SphericalJoint::computeJacobian(MatrixXd &jacobian, int startRow) const {
  const auto r1 = m_bodyA->getPosition();
  const auto r2 = m_bodyB->getPosition();
  const auto A1 = quaternionToRotationMatrix(m_bodyA->getOrientation());
  const auto A2 = quaternionToRotationMatrix(m_bodyB->getOrientation());
  const auto d = r2 + A2 * m_localPointB - r1 - A1 * m_localPointA;

  jacobian.block<1,3>(startRow, m_bodyA->getIndex()*6)      = - 2.0 * d.transpose();
  jacobian.block<1,3>(startRow, m_bodyA->getIndex()*6 + 3)  =   2.0 * d.transpose() * A1 * skew(m_localPointA);
  jacobian.block<1,3>(startRow, m_bodyB->getIndex()*6)      =   2.0 * d.transpose();
  jacobian.block<1,3>(startRow, m_bodyB->getIndex()*6 + 3)  = - 2.0 * d.transpose() * A2 * skew(m_localPointB);
}

void SphericalJoint::computeAccelerationCorrection(VectorXd &gamma, int startRow) const {
  const auto r1 = m_bodyA->getPosition();
  const auto r2 = m_bodyB->getPosition();

  const auto A1 = quaternionToRotationMatrix(m_bodyA->getOrientation());
  const auto A2 = quaternionToRotationMatrix(m_bodyB->getOrientation());

  const auto omega1 = m_bodyA->getAngularVelocity();
  const auto omega2 = m_bodyB->getAngularVelocity();

  const auto r1d = m_bodyA->getLinearVelocity();
  const auto r2d = m_bodyB->getLinearVelocity();

  const auto d = r2 + A2 * m_localPointB - r1 - A1 * m_localPointA;
  auto v = r2d + A2 * skew(omega1) * m_localPointB - r1d - A1 * skew(omega2) * m_localPointA;

  auto vtv = (v.transpose() * v);
  auto par = A2 * skew(omega2) * skew(omega2) * m_localPointB - (A1 * skew(omega1) * skew(omega1) * m_localPointA);

  auto result = - 2 * vtv - 2 * (d.transpose() * par).eval();

  gamma.segment<1>(startRow) = result;
}
} // Proton