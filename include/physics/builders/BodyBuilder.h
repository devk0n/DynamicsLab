#ifndef BODY_BUILDER_H
#define BODY_BUILDER_H

namespace Proton {

class Dynamics;
class Body;

enum class AngleMode {
  Degrees,
  Radians
};

class BodyBuilder {
public:
  BodyBuilder(Dynamics &dynamics, Body* body)
      : m_dynamics(dynamics),
        m_body(body),
        m_angleMode(AngleMode::Degrees),
        m_autoInertia(true) {}

  // Setter for angle mode
  BodyBuilder& setAngleMode(AngleMode mode) {
    m_angleMode = mode;
    return *this;
  }

  BodyBuilder& mass(double mass) {
    m_body->setMass(mass);
    return *this;
  }

  BodyBuilder& inertia(double Ix, double Iy, double Iz) {
    m_body->setInertia({Ix, Iy, Iz});
    return *this;
  }

  BodyBuilder& size(double x, double y, double z) {
    m_body->setSize(Vector3d(x, y, z));
    return *this;
  }

  BodyBuilder& position(double x, double y, double z) {
    m_body->setPosition(Vector3d(x, y, z));
    return *this;
  }

  // Direct quaternion setting remains unchanged.
  BodyBuilder& orientation(double w, double x, double y, double z) {
    m_body->setOrientation({w, x, y, z});
    return *this;
  }

  // Euler-based orientation. Uses the builder's angle mode.
  BodyBuilder& orientation(double x, double y, double z) {
    if (m_angleMode == AngleMode::Degrees)
      m_body->setOrientation(eulerToQuaternionDegrees({x, y, z}));
    else
      m_body->setOrientation(eulerToQuaternion({x, y, z}));
    return *this;
  }

  BodyBuilder& velocity(double x, double y, double z) {
    m_body->setLinearVelocity(Vector3d(x, y, z));
    return *this;
  }

  BodyBuilder& angularVelocity(double x, double y, double z) {
    m_body->setAngularVelocity(Vector3d(x, y, z));
    return *this;
  }

  BodyBuilder& fixed(bool isFixed) {
    m_body->setFixed(isFixed);
    return *this;
  }

  // Finalize and return the pointer to the Body.
  [[nodiscard]] Body* build() const {
    if (m_autoInertia) {
      // Calculate inertia tensor for a solid cuboid
      const Vector3d& size = m_body->getSize();
      double mass = m_body->getMass();

      // For a solid cuboid, the moments of inertia are:
      // Ix = (1/12) * m * (y² + z²)
      // Iy = (1/12) * m * (x² + z²)
      // Iz = (1/12) * m * (x² + y²)
      double Ix = (mass / 12.0) * (size.y() * size.y() + size.z() * size.z());
      double Iy = (mass / 12.0) * (size.x() * size.x() + size.z() * size.z());
      double Iz = (mass / 12.0) * (size.x() * size.x() + size.y() * size.y());

      // Set the calculated inertia tensor
      m_body->setInertia({Ix, Iy, Iz});
    }
    return m_body;
  }


private:
  Dynamics &m_dynamics;
  Body* m_body{nullptr};
  AngleMode m_angleMode;
  bool m_autoInertia;
};
} // Proton

#endif // BODY_BUILDER_H
