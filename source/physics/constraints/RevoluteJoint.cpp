#include "RevoluteJoint.h"

#include <utility>

namespace Proton {
RevoluteJoint::RevoluteJoint(
    Body* bodyA, Vector3d localPointA, Vector3d axis1,
    Body* bodyB, Vector3d localPointB, Vector3d axis2, Vector3d axis3)
      : Constraint(5),
        m_bodyA(bodyA),
        m_bodyB(bodyB),
        m_localPointA(std::move(localPointA)),
        m_localPointB(std::move(localPointB)),
        m_axisA(std::move(axis1)),
        m_axisB(std::move(axis2)),
        m_axisC(std::move(axis3)){}

void RevoluteJoint::computePositionError(VectorXd &phi, int startRow) const {
  auto r1 = m_bodyA->getPosition();
  auto r2 = m_bodyB->getPosition();

  auto A1 = quaternionToRotationMatrix(m_bodyA->getOrientation());
  auto A2 = quaternionToRotationMatrix(m_bodyB->getOrientation());

  // --- 3 constraints for the "spherical" anchor coincidence ---
  phi.segment<3>(startRow) = r1 + A1 * m_localPointA - r2 - A2 * m_localPointB;

  // --- 2 constraints for the revolute axis alignment ---
  phi.segment<1>(startRow + 3) = (A1 * m_axisA).transpose() * (A2 * m_axisB);
  phi.segment<1>(startRow + 4) = (A1 * m_axisA).transpose() * (A2 * m_axisC);
}

void RevoluteJoint::computeJacobian(MatrixXd &jacobian, int startRow) const {
  auto A1 = quaternionToRotationMatrix(m_bodyA->getOrientation());
  auto A2 = quaternionToRotationMatrix(m_bodyB->getOrientation());

  // --- Spherical part (3 constraints) ---
  jacobian.block<3,3>(startRow, m_bodyA->getIndex() * 6)     =  Matrix3d::Identity();
  jacobian.block<3,3>(startRow, m_bodyA->getIndex() * 6 + 3) = -A1 * skew(m_localPointA);
  jacobian.block<3,3>(startRow, m_bodyB->getIndex() * 6)     = -Matrix3d::Identity();
  jacobian.block<3,3>(startRow, m_bodyB->getIndex() * 6 + 3) =  A2 * skew(m_localPointB);

  // --- Axis alignment constraints (4th, 5th) ---
  // 4th constraint: (A1*a1) dot (A2*a2) = 0
  jacobian.block<1,3>(startRow + 3, m_bodyA->getIndex() * 6 + 3) = (A2 * m_axisB).transpose() * (A1 * skew(m_axisA));
  jacobian.block<1,3>(startRow + 3, m_bodyB->getIndex() * 6 + 3) = (A1 * m_axisA).transpose() * (A2 * skew(m_axisB));

  // 5th constraint: (A1 * a1) dot (A2 * a3) = 0
  jacobian.block<1,3>(startRow + 4, m_bodyA->getIndex() * 6 + 3) = (A2 * m_axisC).transpose() * (A1 * skew(m_axisA));
  jacobian.block<1,3>(startRow + 4, m_bodyB->getIndex() * 6 + 3) = (A1 * m_axisA).transpose() * (A2 * skew(m_axisC));
}

void RevoluteJoint::computeAccelerationCorrection(VectorXd &gamma, int startRow) const {
  auto A1 = quaternionToRotationMatrix(m_bodyA->getOrientation());
  auto A2 = quaternionToRotationMatrix(m_bodyB->getOrientation());

  // Angular velocities
  auto omega1 = m_bodyA->getAngularVelocity();
  auto omega2 = m_bodyB->getAngularVelocity();

  // --------------------------------------------------
  // 1) Spherical part (first 3 constraints)
  // --------------------------------------------------
  // Typical "centrifugal" form for anchor coincidence:
  //   A1 * (ω1×(ω1×local1)) - A2 * (ω2×(ω2×local2))
  //
  auto result = A1 * (skew(omega1) * skew(m_localPointA)).eval() * omega1
              - A2 * (skew(omega2) * skew(m_localPointB)).eval() * omega2;
  gamma.segment<3>(startRow) = result;

  // --------------------------------------------------
  // 2) Revolute axis constraints (4th, 5th constraints)
  // --------------------------------------------------
  // Let:
  //   wA1 = A1 * m_axisA  (axis on bodyA in world frame)
  //   wA2 = A2 * m_axisB  (first axis on bodyB in world frame)
  //   wA3 = A2 * m_axisC  (second axis on bodyB in world frame)
  //
  // The velocity-dependent portion of each dot-product constraint
  // (wA1 dot wA2 = 0) and (wA1 dot wA3 = 0) is:
  //
  //    cTerm = wA1·[ ω2×(ω2×wA2 ) ]
  //          + 2 ( ω1×wA1 )·( ω2×wA2 )
  //          + [ ω1×( ω1×wA1 ) ]·wA2
  //
  // (And similarly with wA2 replaced by wA3 for the 5th constraint.)
  // gamma = - (that expression).

  Vector3d wA1 = A1 * m_axisA;
  Vector3d wA2 = A2 * m_axisB;
  Vector3d wA3 = A2 * m_axisC;

  // ---- 4th constraint's velocity term ----
  double cTerm2 = wA1.dot(omega2.cross(omega2.cross(wA2)))
                + 2.0 * (omega1.cross(wA1)).dot(omega2.cross(wA2))
                + (omega1.cross(omega1.cross(wA1))).dot(wA2);

  // ---- 5th constraint's velocity term ----
  double cTerm3 = wA1.dot(omega2.cross(omega2.cross(wA3)))
                + 2.0 * (omega1.cross(wA1)).dot(omega2.cross(wA3))
                + (omega1.cross(omega1.cross(wA1))).dot(wA3);

  gamma[startRow + 3] = -cTerm2;
  gamma[startRow + 4] = -cTerm3;
}
} // Proton
