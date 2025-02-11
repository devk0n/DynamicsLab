#include "RigidBody.h"

#include <utility>
#include "graphics/Mesh.h"

RigidBody::RigidBody(Eigen::Vector3d position,
                     Eigen::Vector4d orientation,
                     Eigen::Matrix3d massMatrix,
                     Eigen::Matrix3d localInertiaTensor,
                     const std::vector<Vertex> &vertices,
                     const std::vector<GLuint> &indices)
    : m_position(std::move(position)),
      m_orientation(std::move(orientation)),
      m_mesh(vertices, indices),
      m_massMatrix(std::move(massMatrix)),
      m_localInertiaTensor(std::move(localInertiaTensor)) {}

const Mesh &RigidBody::getMesh() const {
  return m_mesh;
}

glm::mat4 RigidBody::getModelMatrix() const {
  // Convert Eigen::Vector4d orientation to a rotation matrix
  Eigen::Quaterniond quat(m_orientation[3], m_orientation[0], m_orientation[1], m_orientation[2]);
  Eigen::Matrix4d model = Eigen::Matrix4d::Identity();
  model.block<3, 3>(0, 0) = quat.toRotationMatrix();
  model.block<3, 1>(0, 3) = m_position;

  // Convert Eigen::Matrix4d to glm::mat4
  glm::mat4 glmModel;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      glmModel[i][j] = static_cast<float>(model(j, i));
    }
  }

  // Explicitly set scale to 1.0
  glmModel = glm::scale(glmModel, glm::vec3(1.0f, 1.0f, 1.0f));
  return glmModel;
}

Eigen::Matrix3d RigidBody::getMassMatrix() {
  return m_massMatrix;
}

double RigidBody::getMass() {
  return m_massMatrix.trace() / 3.0;
}

void RigidBody::setLinearVelocity(Eigen::Vector3d linearVelocity) {
  m_linearVelocity = linearVelocity;
}

void RigidBody::setAngularVelocity(Eigen::Vector4d angularVelocity) {
  m_angularVelocity = angularVelocity;
}

const Eigen::Vector3d &RigidBody::getLinearVelocity() const {
  return m_linearVelocity;
}

const Eigen::Vector4d &RigidBody::getAngularVelocity() const {
  return m_angularVelocity;
}
