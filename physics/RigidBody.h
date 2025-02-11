#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include <Eigen/Dense>
#include "graphics/Mesh.h"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>

class RigidBody {
public:
  RigidBody(Eigen::Vector3d position,
            Eigen::Vector4d orientation,
            Eigen::Matrix3d massMatrix,
            Eigen::Matrix3d localInertiaTensor,
            const std::vector<Vertex> &vertices,
            const std::vector<GLuint> &indices);

  // Getters
  [[nodiscard]] glm::mat4 getModelMatrix() const;

  [[nodiscard]] const Mesh &getMesh() const;

  [[nodiscard]] const Eigen::Vector3d &getPosition() const {
    return m_position;
  }

  [[nodiscard]] const Eigen::Vector4d &getOrientation() const {
    return m_orientation;
  }

  [[nodiscard]] const Eigen::Vector3d &getLinearVelocity() const;

  [[nodiscard]] const Eigen::Vector4d &getAngularVelocity() const;

  Eigen::Matrix3d getMassMatrix();

  double getMass();

  // Setters
  void setPosition(const Eigen::Vector3d &position) {
    m_position = position;
  }

  void setOrientation(const Eigen::Vector4d &orientation) {
    m_orientation = orientation;
  }

  void setLinearVelocity(Eigen::Vector3d linearVelocity);

  void setAngularVelocity(Eigen::Vector4d angularVelocity);

private:
  Eigen::Vector3d m_position;
  Eigen::Vector4d m_orientation;

  Eigen::Vector3d m_linearVelocity;
  Eigen::Vector4d m_angularVelocity;

  Mesh m_mesh;

  Eigen::Matrix3d m_massMatrix;
  Eigen::Matrix3d m_localInertiaTensor;
};


#endif // RIGIDBODY_H
