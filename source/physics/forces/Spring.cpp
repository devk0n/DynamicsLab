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

void Spring::apply(double dt) {
  // 1. Transform local points to world space
  Vector3d worldPointA =
    m_bodyA->getPosition() + quaternionToRotationMatrix(m_bodyA->getOrientation()) * m_localPointA;
  Vector3d worldPointB =
    m_bodyB->getPosition() + quaternionToRotationMatrix(m_bodyB->getOrientation()) * m_localPointB;

  // 2. Compute spring direction and length
  Vector3d delta = worldPointB - worldPointA;
  double currentLength = delta.norm();
  if (currentLength < 1e-6) return;  // Avoid division by zero

  Vector3d direction = delta / currentLength;

  // 3. Calculate relative velocity at attachment points
  Vector3d velA = m_bodyA->getLinearVelocity() +
                 m_bodyA->getAngularVelocity().cross(worldPointA - m_bodyA->getPosition());
  Vector3d velB = m_bodyB->getLinearVelocity() +
                 m_bodyB->getAngularVelocity().cross(worldPointB - m_bodyB->getPosition());
  Vector3d relativeVel = velB - velA;

  // 4. Project relative velocity onto spring direction
  double velAlongSpring = relativeVel.dot(direction);

  // 5. Apply Hooke's law (F = -k * (x - L₀)) and damping (F_damp = -d * v)
  double springForce = m_stiffness * (currentLength - m_restLength);
  double dampingForce = m_damping * velAlongSpring;
  double totalForce = springForce + dampingForce;

  Vector3d force = direction * totalForce;

  // 6. Apply equal and opposite forces at the attachment points
  m_bodyA->addForce(force);
  m_bodyB->addForce(-force);

  // 7. Apply torques
  Vector3d torqueA = (worldPointA - m_bodyA->getPosition()).cross(force);
  Vector3d torqueB = (worldPointB - m_bodyB->getPosition()).cross(-force);
  m_bodyA->addTorque(torqueA);
  m_bodyB->addTorque(torqueB);
}
} // Proton