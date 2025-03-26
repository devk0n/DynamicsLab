#ifndef SPRING_FORCE_H
#define SPRING_FORCE_H

#include "Body.h"
#include "ForceGenerator.h"

namespace Proton {

class SpringForce final : public ForceGenerator {
public:
  SpringForce(
      Body* body1,
      Body* body2,
      const double restLength,
      const double stiffness,
      const double damping = 0.1)
      : m_body1(body1),
        m_body2(body2),
        m_restLength(restLength),
        m_stiffness(stiffness),
        m_damping(damping) {}

  void apply(double dt) override {
    Vector3d delta = m_body2->getPosition() - m_body1->getPosition();
    double currentLength = delta.norm();
    if (currentLength < 1e-6) return;

    Vector3d direction = delta / currentLength;

    // Correct Hooke's Law: force pulls towards the rest length.
    double springForce = m_stiffness * (m_restLength - currentLength);

    // Damping force (proportional to relative velocity along the spring direction)
    Vector3d relativeVel = m_body2->getLinearVelocity() - m_body1->getLinearVelocity();
    double dampingForce = m_damping * relativeVel.dot(direction);

    // Total force along the spring direction
    double totalForce = springForce + dampingForce;
    Vector3d force = direction * totalForce;

    // Apply forces so that both bodies are pulled together when the spring is stretched.
    m_body1->addForce(-force);
    m_body2->addForce(force);
  }


private:
  Body* m_body1;
  Body* m_body2;
  double m_restLength;
  double m_stiffness;
  double m_damping;
};
} // Proton

#endif // SPRING_FORCE_H
