#ifndef SPRING_BUILDER_H
#define SPRING_BUILDER_H

#include "Dynamics.h"
#include "Spring.h"

namespace Proton {

class SpringBuilder {
public:
  explicit SpringBuilder(Dynamics &dynamics)
    : m_dynamics(dynamics),
      m_spring(std::make_shared<Spring>()){}

  SpringBuilder& withBodyA(Body* body) {
    m_spring->setBodyA(body);
    return *this;
  }

  SpringBuilder& withBodyB(Body* body) {
    m_spring->setBodyB(body);
    return *this;
  }

  SpringBuilder& withLocalPointA(double x, double y, double z) {
    m_spring->setLocalPointA(Vector3d(x, y, z));
    return *this;
  }

  SpringBuilder& withLocalPointB(double x, double y, double z) {
    m_spring->setLocalPointB(Vector3d(x, y, z));
    return *this;
  }

  SpringBuilder& withRestLength(double length) {
    m_spring->setRestLength(length);
    return *this;
  }

  SpringBuilder& withStiffness(double stiffness) {
    m_spring->setStiffness(stiffness);
    return *this;
  }

  SpringBuilder& withDamping(double damping) {
    m_spring->setDamping(damping);
    return *this;
  }

  std::shared_ptr<Spring> build() {
    m_dynamics.addForceGenerator(m_spring);
    return m_spring;
  }

private:
  Dynamics &m_dynamics;
  std::shared_ptr<Spring> m_spring;
};
} // Proton
#endif // SPRING_BUILDER_H
