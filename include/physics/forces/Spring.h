#ifndef SPRING_FORCE_H
#define SPRING_FORCE_H

#include "Body.h"
#include "ForceElement.h"

namespace Proton {
class Spring final : public ForceElement {
public:
  Spring() = default;

  void computeForceAndJacobian(
      VectorXd& F_ext,
      MatrixXd& K,
      int dof_dq
  ) override;

  void computeDistance();

  [[nodiscard]] Body* getBodyA() const { return m_bodyA; }
  [[nodiscard]] Body* getBodyB() const { return m_bodyB; }
  [[nodiscard]] const Vector3d& getLocalPointA() const { return m_localPointA; }
  [[nodiscard]] const Vector3d& getLocalPointB() const { return m_localPointB; }
  [[nodiscard]] const double& getRestLength() const { return m_restLength; }
  [[nodiscard]] const double& getStiffness() const { return m_stiffness; }
  [[nodiscard]] const double& getDamping() const { return m_damping; }

  void setBodyA(Body* bodyA) { m_bodyA = bodyA; }
  void setBodyB(Body* bodyB) { m_bodyB = bodyB; }
  void setLocalPointA(const Vector3d& localA) { m_localPointA = localA; }
  void setLocalPointB(const Vector3d& localB) { m_localPointB = localB; }
  void setRestLength(const double& restLength) { m_restLength = restLength; }
  void setStiffness(const double& stiffness) { m_stiffness = stiffness; }
  void setDamping(const double& damping) { m_damping = damping; }

private:
  Body* m_bodyA{nullptr};
  Body* m_bodyB{nullptr};
  Vector3d m_localPointA{0, 0, 0};
  Vector3d m_localPointB{0, 0, 0};
  double m_restLength{0};
  double m_stiffness{0};
  double m_damping{0};
};
} // Proton
#endif // SPRING_FORCE_H
