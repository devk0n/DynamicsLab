#include "DistanceConstraint.h"

namespace Proton {

DistanceConstraint::DistanceConstraint(
    Body* body1,
    Body* body2)
    : Constraint(1),
      m_body1(body1),
      m_body2(body2) {

  Vector3d d = m_body2->getPosition() - m_body1->getPosition();

  m_distance = d.norm();

}

void DistanceConstraint::computePositionError(VectorXd &phi, const int startRow) const {
  // Relative position
  Vector3d d = m_body2->getPosition() - m_body1->getPosition();

  // Constraint equation
  phi[startRow] = (d.transpose() * d) - (m_distance * m_distance);
}

void DistanceConstraint::computeJacobian(MatrixXd &jacobian, const int startRow) const {
  // Relative position
  Vector3d d = m_body2->getPosition() - m_body1->getPosition();

  // Jacobian matrix
  jacobian.block<1, 3>(startRow, m_body1->getIndex() * 6) = - 2 * d.transpose();
  jacobian.block<1, 3>(startRow, m_body2->getIndex() * 6) =   2 * d.transpose();
}

void DistanceConstraint::computeAccelerationCorrection(VectorXd &gamma, const int startRow) const {
  Vector3d v = m_body2->getLinearVelocity() - m_body1->getLinearVelocity();

  gamma[startRow] = - 2.0 * v.transpose() * v;
}
} // Proton
