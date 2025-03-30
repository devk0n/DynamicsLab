#include "Spring.h"

namespace Proton {
Spring::Spring(
      Body* bodyA, Vector3d localPointA,
      Body* bodyB, Vector3d localPointB,
      const double restLength,
      const double stiffness)
      : m_bodyA(bodyA),
        m_bodyB(bodyB),
        m_localPointA(std::move(localPointA)),
        m_localPointB(std::move(localPointB)),
        m_restLength(restLength),
        m_stiffness(stiffness) {}

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

  // 3. Apply Hooke's law (F = -k * (x - L₀))
  double springForce = m_stiffness * (currentLength - m_restLength);
  Vector3d force = direction * springForce;

  // 4. Apply equal and opposite forces at the attachment points
  m_bodyA->addForce(force);
  m_bodyB->addForce(-force);

  Vector3d torqueA = (worldPointA - m_bodyA->getPosition()).cross(force);
  Vector3d torqueB = (worldPointB - m_bodyB->getPosition()).cross(-force);
  m_bodyA->addTorque(torqueA);
  m_bodyB->addTorque(torqueB);

}
} // Proton