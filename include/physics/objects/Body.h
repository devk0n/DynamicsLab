#ifndef BODY_H
#define BODY_H

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "Proton.h"

namespace Proton {
class Body {
public:
  Body(
      UniqueID ID,
      int index,
      double mass,
      const Vector3d& inertia,
      const Vector3d& position,
      const Vector4d& orientation)
        : m_ID(ID),
          m_index(index),
          m_mass(mass),
          m_inverseMass(mass > std::numeric_limits<double>::epsilon() ? 1.0 / mass : 0.0),
          m_inertia(inertia),
          m_inverseInertia(calculateInverseInertia(inertia)),
          m_position(position),
          m_orientation(orientation.normalized()) {
    updateInertiaWorld();
  }

  // Update inertia tensor in world space
  void updateInertiaWorld() {
    m_inverseInertiaWorld = Proton::updateInertiaWorld(
      m_orientation, m_inverseInertia);
  }

  // Getters
  UniqueID getID() const noexcept { return m_ID; }
  int getIndex() const noexcept { return m_index; }

  const Vector3d &getPosition() const noexcept { return m_position; }
  const Vector3d &getLinearVelocity() const noexcept { return m_velocity; }
  const Vector4d &getOrientation() const noexcept { return m_orientation; }
  const Vector3d &getAngularVelocity() const noexcept { return m_angularVelocity; }

  double getMass() const noexcept { return m_mass; }
  double getInverseMass() const noexcept { return m_inverseMass; }
  const Vector3d &getInertia() const noexcept { return m_inertia; }
  const Vector3d &getInverseInertia() const noexcept { return m_inverseInertia; }
  const Matrix3d &getInertiaWorld() const noexcept { return m_inverseInertiaWorld; }

  // Setters
  void setPosition(const Vector3d &position) noexcept { m_position = position; }
  void setLinearVelocity(const Vector3d &velocity) noexcept { m_velocity = velocity; }
  void setOrientation(const Vector4d &orientation) noexcept {
    m_orientation = orientation.normalized();
    updateInertiaWorld();
  }

  void setAngularVelocity(const Vector3d &angularVelocity) noexcept {
    m_angularVelocity = angularVelocity;
  }

  // Force operations
  void addForce(const Vector3d &force) noexcept { m_force.noalias() += force; }
  void clearForces() noexcept { m_force.setZero(); }
  const Vector3d &getForce() const noexcept { return m_force; }

  // Torque operations
  void addTorque(const Vector3d &torque) noexcept { m_torque.noalias() += torque; }
  void clearTorque() noexcept { m_torque.setZero(); }
  const Vector3d &getTorque() const noexcept { return m_torque; }

  // Configuration
  void setFixed(bool fixed) noexcept { m_fixed = fixed; }
  bool isFixed() const noexcept { return m_fixed; }
  void setSize(const Vector3d &size) noexcept { m_size = size; }
  const Vector3d &getSize() const noexcept { return m_size; }

  // Visualization conversion
  [[nodiscard]] glm::vec3 getPositionVec3() const noexcept {
    return glm::vec3(
      static_cast<float>(m_position.x()),
      static_cast<float>(m_position.y()),
      static_cast<float>(m_position.z())
    );
  }

  [[nodiscard]] glm::vec3 getSizeVec3() const noexcept {
    return glm::vec3(
      static_cast<float>(m_size.x()),
      static_cast<float>(m_size.y()),
      static_cast<float>(m_size.z())
    );
  }

  [[nodiscard]] glm::quat getOrientationQuat() const noexcept {
    return glm::quat(
      static_cast<float>(m_orientation.x()),
      static_cast<float>(m_orientation.y()),
      static_cast<float>(m_orientation.z()),
      static_cast<float>(m_orientation.w())
    );
  }

private:
  static Vector3d calculateInverseInertia(const Vector3d &inertia) noexcept {
    return Vector3d(
      inertia.x() > 0.0 ? 1.0 / inertia.x() : 0.0,
      inertia.y() > 0.0 ? 1.0 / inertia.y() : 0.0,
      inertia.z() > 0.0 ? 1.0 / inertia.z() : 0.0
    );
  }

  UniqueID m_ID;
  int m_index;

  // Physical properties
  Vector3d m_size = Vector3d::Ones();
  double m_mass = 0.0;
  double m_inverseMass = 0.0;
  Vector3d m_inertia = Vector3d::Zero();
  Vector3d m_inverseInertia = Vector3d::Zero();
  Matrix3d m_inverseInertiaWorld = Matrix3d::Zero();

  // State variables
  Vector3d m_position = Vector3d::Zero();
  Vector3d m_velocity = Vector3d::Zero();
  Vector4d m_orientation = Vector4d(1, 0, 0, 0);
  Vector3d m_angularVelocity = Vector3d::Zero();

  // Force accumulators
  Vector3d m_force = Vector3d::Zero();
  Vector3d m_torque = Vector3d::Zero();

  bool m_fixed = false;
};
} // Proton

#endif // BODY_H
