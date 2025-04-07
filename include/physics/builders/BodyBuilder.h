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
  BodyBuilder(Dynamics& dynamics, Body* body)
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

  BodyBuilder& geometryType(GeometryType type) {
    m_body->setGeometryType(type);
    return *this;
  }

  BodyBuilder& color(float r, float g, float b, float a = 1.0f) {
    m_body->setColor(glm::vec4(r, g, b, a));
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

  BodyBuilder& autoInertia(bool autoInertia) {
    m_autoInertia = autoInertia;
    return *this;
  }

  // Finalize and return the pointer to the Body.
  [[nodiscard]] Body* build() const {
    if (m_autoInertia) {
      const Vector3d& size = m_body->getSize();
      double mass = m_body->getMass();
      double Ix = 0, Iy = 0, Iz = 0;

      switch (m_body->getGeometryType()) {
        case GeometryType::Cube: {
          // Solid box
          Ix = mass / 12.0 * (size.y() * size.y() + size.z() * size.z());
          Iy = mass / 12.0 * (size.x() * size.x() + size.z() * size.z());
          Iz = mass / 12.0 * (size.x() * size.x() + size.y() * size.y());
          break;
        }

        case GeometryType::Cylinder: {
          // Solid cylinder (aligned along Y-axis by default)
          double r = 0.5 * (size.x() + size.z()); // average radius (XZ plane)
          double h = size.y(); // height

          Ix = Iz = (1.0 / 12.0) * mass * (3.0 * r * r + h * h);
          Iy = 0.5 * mass * r * r;
          break;
        }

        // Extend this with other shapes
        default:
          LOG_WARN("Unknown shape type, falling back to cube inertia.");
        Ix = mass / 12.0 * (size.y() * size.y() + size.z() * size.z());
        Iy = mass / 12.0 * (size.x() * size.x() + size.z() * size.z());
        Iz = mass / 12.0 * (size.x() * size.x() + size.y() * size.y());
        break;
      }

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
} // namespace Proton
#endif // BODY_BUILDER_H
