#ifndef SPRING_FORCE_H
#define SPRING_FORCE_H

#include <utility>

#include "Body.h"
#include "ForceElement.h"

namespace Proton {

class Spring final : public ForceElement {
public:
  Spring(
      Body* bodyA, Vector3d localPointA,
      Body* bodyB, Vector3d localPointB,
      double restLength,
      double stiffness,
      double damping
  );

  Spring();

  void computeDistance();

  void computeForceAndJacobian(
      Eigen::VectorXd& F_ext,
      Eigen::MatrixXd& K,
      int dof_dq
  ) override;

  [[nodiscard]] Body* getBodyA() const { return m_bodyA; }
  [[nodiscard]] Body* getBodyB() const { return m_bodyB; }
  [[nodiscard]] Vector3d getLocalPointA() const { return m_localPointA; }
  [[nodiscard]] Vector3d getLocalPointB() const { return m_localPointB; }
  [[nodiscard]] double getRestLength() const { return m_restLength; }
  [[nodiscard]] double getStiffness() const { return m_stiffness; }
  [[nodiscard]] double getDamping() const { return m_damping; }

  void setBodyA(Body* bodyA) { m_bodyA = bodyA; }
  void setBodyB(Body* bodyB) { m_bodyB = bodyB; }
  void setLocalPointA(Vector3d localA) { m_localPointA = std::move(localA); }
  void setLocalPointB(Vector3d localB) { m_localPointB = std::move(localB); }
  void setRestLength(double restLength) { m_restLength = restLength; }
  void setStiffness(double stiffness) { m_stiffness = stiffness; }
  void setDamping(double damping) { m_damping = damping; }

private:
  Body* m_bodyA{nullptr};
  Body* m_bodyB{nullptr};
  Vector3d m_localPointA{0, 0, 0};
  Vector3d m_localPointB{0, 0, 0};
  double m_restLength = 0;
  double m_stiffness = 0;
  double m_damping = 0;
};
} // Proton

#endif // SPRING_FORCE_H
