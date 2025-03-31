#include "RevoluteJoint.h"

#include <utility>

namespace Proton {
RevoluteJoint::RevoluteJoint(
    Body* bodyA, Vector3d localPointA, Vector3d axisA,
    Body* bodyB, Vector3d localPointB, Vector3d axisB)
      : Constraint(5),
        m_bodyA(bodyA),
        m_bodyB(bodyB),
        m_localPointA(std::move(localPointA)),
        m_localPointB(std::move(localPointB)),
        m_axisA(std::move(axisA)),
        m_axisB(std::move(axisB)) {
  calculateConstraintAxes();
}

void RevoluteJoint::calculateConstraintAxes() {
  // Find a vector not parallel to m_axisB
  auto temp = Vector3d(1, 0, 0);
  if (m_axisB.cross(temp).norm() < 1e-6) {
    temp = Vector3d(0, 1, 0);
  }

  m_axisN = m_axisB.cross(temp).normalized();
  m_axisM = m_axisB.cross(m_axisN).normalized();
}

void RevoluteJoint::computePositionError(VectorXd &phi, int startRow) const {
  const auto& rA = m_bodyA->getPosition();
  const auto& rB = m_bodyB->getPosition();
  const auto A1 = quaternionToRotationMatrix(m_bodyA->getOrientation());
  const auto A2 = quaternionToRotationMatrix(m_bodyB->getOrientation());

  // Point constraint (3 rows)
  phi.segment<3>(startRow) =
      rA + A1 * m_localPointA - rB - A2 * m_localPointB;

  // Axis alignment constraints (2 rows)
  const Vector3d worldAxisA = A1 * m_axisA;
  const Vector3d worldAxisM = A2 * m_axisM;
  const Vector3d worldAxisN = A2 * m_axisN;

  phi[startRow + 3] = worldAxisA.dot(worldAxisM);  // Should be 0
  phi[startRow + 4] = worldAxisA.dot(worldAxisN);  // Should be 0
}

void RevoluteJoint::computeJacobian(MatrixXd &jacobian, int startRow) const {
  const auto A1 = quaternionToRotationMatrix(m_bodyA->getOrientation());
  const auto A2 = quaternionToRotationMatrix(m_bodyB->getOrientation());

  // World space points and axes
  const Vector3d worldPointA = A1 * m_localPointA;
  const Vector3d worldPointB = A2 * m_localPointB;
  const Vector3d worldAxisA = A1 * m_axisA;
  const Vector3d worldAxisM = A2 * m_axisM;
  const Vector3d worldAxisN = A2 * m_axisN;

  // Point constraint Jacobian (3 rows)
  jacobian.block<3,3>(startRow, m_bodyA->getIndex() * 6) = Matrix3d::Identity();
  jacobian.block<3,3>(startRow, m_bodyA->getIndex() * 6 + 3) = -skew(worldPointA);
  jacobian.block<3,3>(startRow, m_bodyB->getIndex() * 6) = -Matrix3d::Identity();
  jacobian.block<3,3>(startRow, m_bodyB->getIndex() * 6 + 3) = skew(worldPointB);

  // Perpendicularity constraints Jacobian (2 rows)
  // d(axisA·axisM)/dθ
  jacobian.block<1,3>(startRow + 3, m_bodyA->getIndex() * 6 + 3) =
      -worldAxisM.transpose() * skew(worldAxisA);
  jacobian.block<1,3>(startRow + 3, m_bodyB->getIndex() * 6 + 3) =
      -worldAxisA.transpose() * skew(worldAxisM);

  // d(axisA·axisN)/dθ
  jacobian.block<1,3>(startRow + 4, m_bodyA->getIndex() * 6 + 3) =
      -worldAxisN.transpose() * skew(worldAxisA);
  jacobian.block<1,3>(startRow + 4, m_bodyB->getIndex() * 6 + 3) =
      -worldAxisA.transpose() * skew(worldAxisN);
}

void RevoluteJoint::computeAccelerationCorrection(VectorXd &gamma, int startRow) const {
  const auto A1 = quaternionToRotationMatrix(m_bodyA->getOrientation());
  const auto A2 = quaternionToRotationMatrix(m_bodyB->getOrientation());

  const Vector3d omega1 = m_bodyA->getAngularVelocity();
  const Vector3d omega2 = m_bodyB->getAngularVelocity();

  // World space quantities
  const Vector3d r1 = A1 * m_localPointA;
  const Vector3d r2 = A2 * m_localPointB;
  const Vector3d worldAxisA = A1 * m_axisA;
  const Vector3d worldAxisM = A2 * m_axisM;
  const Vector3d worldAxisN = A2 * m_axisN;

  // Point constraint acceleration (3 rows)
  gamma.segment<3>(startRow) =
      omega1.cross(omega1.cross(r1)) - omega2.cross(omega2.cross(r2));

  // Perpendicularity constraints acceleration (2 rows)
  // γ = -2(ω₁×a)ᵀ(ω₂×m) - aᵀ(ω₂×(ω₂×m)) - (ω₁×(ω₁×a))ᵀm
  gamma[startRow + 3] =
      -2.0 * omega1.cross(worldAxisA).dot(omega2.cross(worldAxisM))
      - worldAxisA.dot(omega2.cross(omega2.cross(worldAxisM)))
      - omega1.cross(omega1.cross(worldAxisA)).dot(worldAxisM);

  gamma[startRow + 4] =
      -2.0 * omega1.cross(worldAxisA).dot(omega2.cross(worldAxisN))
      - worldAxisA.dot(omega2.cross(omega2.cross(worldAxisN)))
      - omega1.cross(omega1.cross(worldAxisA)).dot(worldAxisN);
}
} // Proton
