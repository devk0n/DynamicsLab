#include "UniversalJoint.h"

#include <utility>

namespace Proton {
UniversalJoint::UniversalJoint(
    Body* body1, Vector3d local1, Vector3d axis1,
    Body* body2, Vector3d local2, Vector3d axis2)
      : Constraint(4),
        m_body1(body1),
        m_body2(body2),
        m_local1(std::move(local1)),
        m_local2(std::move(local2)),
        m_axis1(std::move(axis1)),
        m_axis2(std::move(axis2)) {}

void UniversalJoint::computePositionError(VectorXd &phi, int startRow) const {
  auto r1 = m_body1->getPosition();
  auto r2 = m_body2->getPosition();

  auto A1 = quaternionToRotationMatrix(m_body1->getOrientation());
  auto A2 = quaternionToRotationMatrix(m_body2->getOrientation());

  phi.segment<3>(startRow) = r1 + A1 * m_local1 - r2 - A2 * m_local2;
  phi.segment<1>(startRow + 3) = (A1 * m_axis1).eval().transpose() * (A2 * m_axis2).eval();
}

void UniversalJoint::computeJacobian(MatrixXd &jacobian, int startRow) const {
  auto A1 = quaternionToRotationMatrix(m_body1->getOrientation());
  auto A2 = quaternionToRotationMatrix(m_body2->getOrientation());

  // Jacobian matrix Spherical
  jacobian.block<3,3>(startRow, m_body1->getIndex() * 6)     =   Matrix3d::Identity();
  jacobian.block<3,3>(startRow, m_body1->getIndex() * 6 + 3) = - A1 * skew(m_local1);
  jacobian.block<3,3>(startRow, m_body2->getIndex() * 6)     = - Matrix3d::Identity();
  jacobian.block<3,3>(startRow, m_body2->getIndex() * 6 + 3) =   A2 * skew(m_local2);

  // Jacobian matrix Universal
  jacobian.block<1,3>(startRow + 3, m_body1->getIndex() * 6 + 3) = (A2 * m_axis2).transpose() * (A1 * skew(m_axis1));
  jacobian.block<1,3>(startRow + 3, m_body2->getIndex() * 6 + 3) = (A1 * m_axis1).transpose() * (A2 * skew(m_axis2));

}

void UniversalJoint::computeAccelerationCorrection(VectorXd &gamma, int startRow) const {
  auto A1 = quaternionToRotationMatrix(m_body1->getOrientation());
  auto A2 = quaternionToRotationMatrix(m_body2->getOrientation());

  auto omega1 = m_body1->getAngularVelocity();
  auto omega2 = m_body2->getAngularVelocity();

  auto result = A1 * (skew(omega1) * skew(m_local1)).eval() * omega1 - A2 * (skew(omega2) * skew(m_local2)).eval() * omega2;
  gamma.segment<3>(startRow) = result;

  Vector3d a1 = A1 * m_axis1;
  Vector3d a2 = A2 * m_axis2;

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