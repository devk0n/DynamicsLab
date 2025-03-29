#include "UniversalJoint.h"

#include <utility>

namespace Proton {
UniversalJoint::UniversalJoint(
    Body* bodyA, Vector3d localPointA, Vector3d axisA,
    Body* bodyB, Vector3d localPointB, Vector3d axisB)
      : Constraint(4),
        m_bodyA(bodyA),
        m_bodyB(bodyB),
        m_localPointA(std::move(localPointA)),
        m_localPointB(std::move(localPointB)),
        m_axisA(std::move(axisA)),
        m_axisB(std::move(axisB)) {}

void UniversalJoint::computePositionError(VectorXd &phi, int startRow) const {
  auto r1 = m_bodyA->getPosition();
  auto r2 = m_bodyB->getPosition();

  auto A1 = quaternionToRotationMatrix(m_bodyA->getOrientation());
  auto A2 = quaternionToRotationMatrix(m_bodyB->getOrientation());

  phi.segment<3>(startRow) = r1 + A1 * m_localPointA - r2 - A2 * m_localPointB;
  phi.segment<1>(startRow + 3) = (A1 * m_axisA).eval().transpose() * (A2 * m_axisB).eval();
}

void UniversalJoint::computeJacobian(MatrixXd &jacobian, int startRow) const {
  auto A1 = quaternionToRotationMatrix(m_bodyA->getOrientation());
  auto A2 = quaternionToRotationMatrix(m_bodyB->getOrientation());

  // Jacobian matrix Spherical
  jacobian.block<3,3>(startRow, m_bodyA->getIndex() * 6)     =   Matrix3d::Identity();
  jacobian.block<3,3>(startRow, m_bodyA->getIndex() * 6 + 3) = - A1 * skew(m_localPointA);
  jacobian.block<3,3>(startRow, m_bodyB->getIndex() * 6)     = - Matrix3d::Identity();
  jacobian.block<3,3>(startRow, m_bodyB->getIndex() * 6 + 3) =   A2 * skew(m_localPointB);

  // Jacobian matrix Universal
  jacobian.block<1,3>(startRow + 3, m_bodyA->getIndex() * 6 + 3) = (A2 * m_axisB).transpose() * (A1 * skew(m_axisA));
  jacobian.block<1,3>(startRow + 3, m_bodyB->getIndex() * 6 + 3) = (A1 * m_axisA).transpose() * (A2 * skew(m_axisB));

}

void UniversalJoint::computeAccelerationCorrection(VectorXd &gamma, int startRow) const {
  auto A1 = quaternionToRotationMatrix(m_bodyA->getOrientation());
  auto A2 = quaternionToRotationMatrix(m_bodyB->getOrientation());

  auto omega1 = m_bodyA->getAngularVelocity();
  auto omega2 = m_bodyB->getAngularVelocity();

  auto result = A1 * (skew(omega1) * skew(m_localPointA)).eval() * omega1 - A2 * (skew(omega2) * skew(m_localPointB)).eval() * omega2;
  gamma.segment<3>(startRow) = result;

  Vector3d a1 = A1 * m_axisA;
  Vector3d a2 = A2 * m_axisB;

  double cTerm = 0.0;

  //  a1 dot [ ω2 × ( ω2 × a2 ) ]
  cTerm += a1.dot(omega2.cross(omega2.cross(a2)));

  //  + 2 ( ω1×a1 ) dot ( ω2×a2 )
  cTerm += 2.0 * (omega1.cross(a1)).dot(omega2.cross(a2));

  //  + [ ω1 × ( ω1 × a1 ) ] dot a2
  cTerm += (omega1.cross(omega1.cross(a1))).dot(a2);

  // gamma(4th constraint) = - (that expression)
  gamma[startRow + 3] = -cTerm;
}
} // Proton