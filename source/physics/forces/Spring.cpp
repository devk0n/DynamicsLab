#include "Spring.h"

namespace Proton {

void Spring::computeDistance() {
  // Compute each anchor’s world‑space position
  const auto& A1 = quaternionToRotationMatrix(m_bodyA->getOrientation());
  const auto& A2 = quaternionToRotationMatrix(m_bodyB->getOrientation());
  const auto& world1 = m_bodyA->getPosition() + A1 * m_localPointA;
  const auto& world2 = m_bodyB->getPosition() + A2 * m_localPointB;

  // Initialize the resting distance automatically
  m_restLength = (world2 - world1).norm();
}

void Spring::computeForceAndJacobian(
    VectorXd& F_ext,
    MatrixXd& K,
    int dof_dq)
{
    if (!m_bodyA || !m_bodyB) return;

    // Get world-space attachment points
    const auto& RA = quaternionToRotationMatrix(m_bodyA->getOrientation());
    const auto& RB = quaternionToRotationMatrix(m_bodyB->getOrientation());
    const auto& rA_world = RA * m_localPointA;
    const auto& rB_world = RB * m_localPointB;
    const auto& worldA = m_bodyA->getPosition() + rA_world;
    const auto& worldB = m_bodyB->getPosition() + rB_world;

    // Spring vector and its properties
    Vector3d r = worldB - worldA;
    double L = r.norm();
    if (L < 1e-6) return; // Avoid division by zero

    Vector3d n = r / L;
    double deltaL = L - m_restLength;

    // 1. Spring force calculation with numerical safety
    Vector3d springForce = -m_stiffness * deltaL * n;

    // 2. Damping force with improved numerical stability
    Vector3d velA = m_bodyA->getLinearVelocity() + m_bodyA->getAngularVelocity().cross(rA_world);
    Vector3d velB = m_bodyB->getLinearVelocity() + m_bodyB->getAngularVelocity().cross(rB_world);
    Vector3d relVel = velB - velA;

    // Project velocity onto spring axis for stable damping
    double velAlongSpring = relVel.dot(n);
    Vector3d dampingForce = -m_damping * velAlongSpring * n;

    Vector3d totalForce = springForce + dampingForce;

    // 3. Torque calculation with stability enhancements
    Vector3d torqueA = rA_world.cross(-totalForce);
    Vector3d torqueB = rB_world.cross(totalForce);

    // Apply forces and torques
    int iA = m_bodyA->getIndex() * 6;
    int iB = m_bodyB->getIndex() * 6;

    F_ext.segment<3>(iA)     -= totalForce;
    F_ext.segment<3>(iB)     += totalForce;
    F_ext.segment<3>(iA + 3) +=    torqueA;
    F_ext.segment<3>(iB + 3) +=    torqueB;

    // 4. Improved Jacobian calculation
    Matrix3d I = Matrix3d::Identity();
    Matrix3d outer_nn = n * n.transpose();

    // Spring stiffness term
    Matrix3d dFdx = -m_stiffness * (outer_nn + deltaL/L * (I - outer_nn));

    // Damping term
    Matrix3d dFdv = -m_damping * outer_nn;

    // Skew matrices
    Matrix3d rAx = skew(rA_world);
    Matrix3d rBx = skew(rB_world);

    // Jacobian blocks
    Matrix6d KAA = Matrix6d::Zero();
    Matrix6d KBB = Matrix6d::Zero();
    Matrix6d KAB = Matrix6d::Zero();
    Matrix6d KBA = Matrix6d::Zero();

    // Linear terms
    KAA.topLeftCorner<3,3>() =   dFdx;
    KBB.topLeftCorner<3,3>() =   dFdx;
    KAB.topLeftCorner<3,3>() = - dFdx;
    KBA.topLeftCorner<3,3>() = - dFdx;

    // Angular terms with stabilization
    KAA.topRightCorner<3,3>()   = - dFdx * rAx;
    KAA.bottomLeftCorner<3,3>() = - rAx.transpose() * dFdx;
    KBB.topRightCorner<3,3>()   =   dFdx * rBx;
    KBB.bottomLeftCorner<3,3>() =   rBx.transpose() * dFdx;

    // Cross terms
    KAB.topRightCorner<3,3>()   =   dFdx * rBx;
    KAB.bottomLeftCorner<3,3>() = - rAx.transpose() * dFdx;
    KBA.topRightCorner<3,3>()   = - dFdx * rAx;
    KBA.bottomLeftCorner<3,3>() =   rBx.transpose() * dFdx;

    // Apply to global matrix
    K.block<6,6>(iA, iA) += KAA;
    K.block<6,6>(iB, iB) += KBB;
    K.block<6,6>(iA, iB) += KAB;
    K.block<6,6>(iB, iA) += KBA;
}

} // Proton