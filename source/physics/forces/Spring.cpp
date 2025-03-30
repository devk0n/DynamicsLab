#include "Spring.h"

namespace Proton {
Spring::Spring(
      Body* bodyA, Vector3d localPointA,
      Body* bodyB, Vector3d localPointB,
      const double restLength,
      const double stiffness,
      const double damping)
      : m_bodyA(bodyA),
        m_bodyB(bodyB),
        m_localPointA(std::move(localPointA)),
        m_localPointB(std::move(localPointB)),
        m_restLength(restLength),
        m_stiffness(stiffness),
        m_damping(damping) {}

Spring::Spring()
    : m_bodyA(nullptr),
      m_bodyB(nullptr) {}

void Spring::computeForceAndJacobian(
    Eigen::VectorXd& F_ext,
    Eigen::MatrixXd& K,
    int dof_dq
) {
  if (!m_bodyA || !m_bodyB) return;

  const auto& posA = m_bodyA->getPosition();
  const auto& posB = m_bodyB->getPosition();
  const auto& oriA = m_bodyA->getOrientation();
  const auto& oriB = m_bodyB->getOrientation();

  Matrix3d RA = quaternionToRotationMatrix(oriA);
  Matrix3d RB = quaternionToRotationMatrix(oriB);

  Vector3d rA = RA * m_localPointA;
  Vector3d rB = RB * m_localPointB;
  Vector3d worldPointA = posA + rA;
  Vector3d worldPointB = posB + rB;

  Vector3d r = worldPointB - worldPointA;
  double L = r.norm();
  if (L < 1e-6) return;

  Vector3d n = r / L;
  double deltaL = L - m_restLength;

  // Spring force
  Vector3d force = -m_stiffness * deltaL * n;

  // Damping
  Vector3d velA = m_bodyA->getLinearVelocity() + m_bodyA->getAngularVelocity().cross(rA);
  Vector3d velB = m_bodyB->getLinearVelocity() + m_bodyB->getAngularVelocity().cross(rB);
  Vector3d relVel = velB - velA;
  double velAlongSpring = relVel.dot(n);

  Vector3d dampingForce = -m_damping * velAlongSpring * n;
  Vector3d totalForce = force + dampingForce;

  int iA = m_bodyA->getIndex() * 6;
  int iB = m_bodyB->getIndex() * 6;

  // Apply forces
  F_ext.segment<3>(iA) -= totalForce;
  F_ext.segment<3>(iB) += totalForce;

  // Apply torques
  Vector3d torqueA = rA.cross(-totalForce); // Vector3d torqueA = 0.1 * rA.cross(-totalForce);
  Vector3d torqueB = rB.cross(totalForce);  // Vector3d torqueB = 0.1 * rB.cross(totalForce);
  F_ext.segment<3>(iA + 3) += torqueA;
  F_ext.segment<3>(iB + 3) += torqueB;

  // Jacobians
  Matrix3d I = Matrix3d::Identity();
  Matrix3d outer = n * n.transpose();
  Matrix3d dFdx = -m_stiffness * (outer + ((deltaL / L) * (I - outer)));

  // Skew matrices
  Matrix3d rAx = skew(rA);
  Matrix3d rBx = skew(rB);

  // Full 6x6 Jacobian block per body
  Matrix6d KAA = Matrix6d::Zero();
  Matrix6d KBB = Matrix6d::Zero();
  Matrix6d KAB = Matrix6d::Zero();
  Matrix6d KBA = Matrix6d::Zero();

  // Linear-linear
  KAA.topLeftCorner<3,3>() =  dFdx;
  KBB.topLeftCorner<3,3>() =  dFdx;
  KAB.topLeftCorner<3,3>() = -dFdx;
  KBA.topLeftCorner<3,3>() = -dFdx;

  // Linear-angular and angular-linear
  KAA.topRightCorner<3,3>()     = -dFdx * rAx;
  KAA.bottomLeftCorner<3,3>()   = -rAx.transpose() * dFdx;

  KBB.topRightCorner<3,3>()     =  dFdx * rBx;
  KBB.bottomLeftCorner<3,3>()   =  rBx.transpose() * dFdx;

  KAB.topRightCorner<3,3>()     =  dFdx * rBx;
  KAB.bottomLeftCorner<3,3>()   = -rAx.transpose() * dFdx;

  KBA.topRightCorner<3,3>()     = -dFdx * rAx;
  KBA.bottomLeftCorner<3,3>()   =  rBx.transpose() * dFdx;

  // Angular-angular
  KAA.bottomRightCorner<3,3>() = rAx.transpose() * dFdx * rAx;
  KBB.bottomRightCorner<3,3>() = rBx.transpose() * dFdx * rBx;
  KAB.bottomRightCorner<3,3>() = -rAx.transpose() * dFdx * rBx;
  KBA.bottomRightCorner<3,3>() = -rBx.transpose() * dFdx * rAx;

  // Apply blocks to global K
  K.block<6,6>(iA, iA) += KAA;
  K.block<6,6>(iB, iB) += KBB;
  K.block<6,6>(iA, iB) += KAB;
  K.block<6,6>(iB, iA) += KBA;
}

} // Proton