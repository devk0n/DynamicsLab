#include "DistanceConstraint.h"

namespace Proton {

DistanceConstraint::DistanceConstraint(
    Body* bodyA,
    Body* bodyB)
    : Constraint(1),
      m_bodyA(bodyA),
      m_bodyB(bodyB) {
  computeDistance();
}

void DistanceConstraint::computeDistance() {
  const auto &d = m_bodyB->getPosition() - m_bodyA->getPosition();
  m_distance = d.norm();
}

void DistanceConstraint::computePositionError(VectorXd &phi, const int startRow) const {
  // Relative position
  const auto &d = m_bodyB->getPosition() - m_bodyA->getPosition();

  // Constraint equation
  phi[startRow] = (d.transpose() * d) - (m_distance * m_distance);
}

void DistanceConstraint::computeJacobian(MatrixXd &jacobian, const int startRow) const {
  // Relative position

  const auto &d = m_bodyB->getPosition() - m_bodyA->getPosition();

  // Jacobian matrix
  jacobian.block<1, 3>(startRow, m_bodyA->getIndex() * 6) = - 2 * d.transpose();
  jacobian.block<1, 3>(startRow, m_bodyB->getIndex() * 6) =   2 * d.transpose();
}

void DistanceConstraint::computeAccelerationCorrection(VectorXd &gamma, const int startRow) const {
  const auto &v = m_bodyB->getLinearVelocity() - m_bodyA->getLinearVelocity();

  gamma[startRow] = - 2.0 * v.transpose() * v;
}
} // Proton
