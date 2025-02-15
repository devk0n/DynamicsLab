#include "RigidBody.h"

#include <utility>
#include "graphics/Mesh.h"

RigidBody::RigidBody(Eigen::Vector3d initialPosition,
                     Eigen::Matrix3d massMatrix,
                     const std::vector<Vertex> &vertices,
                     const std::vector<GLuint> &indices,
                     const glm::vec3 color,
                     Eigen::Vector3d initialLinearVelocity)
  : color(color),
    m_position(std::move(initialPosition)),
    m_linearVelocity(std::move(initialLinearVelocity)),
    m_massMatrix(std::move(massMatrix)),
    m_mesh(vertices, indices),
    m_initialPosition(std::move(initialPosition)),
    m_initialLinearVelocity(std::move(initialLinearVelocity)) {
}

glm::mat4 RigidBody::getModelMatrix() const {
  // Convert Eigen::Vector4d orientation to a rotation matrix
  const Eigen::Quaterniond quat(1, 0, 0, 0);
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
  glmModel = scale(glmModel, glm::vec3(1.0f, 1.0f, 1.0f));
  return glmModel;
}

void RigidBody::reset() {
  m_position = m_initialPosition;
  m_linearVelocity = m_initialLinearVelocity;
}

double RigidBody::getMass() const {
  return m_massMatrix.trace() / 3.0;
}

Eigen::Matrix3d &RigidBody::getMassMatrix() {
  return m_massMatrix;
}

void RigidBody::setPosition(const Eigen::Vector3d &position) {
  m_position = position;
}

void RigidBody::setLinearVelocity(const Eigen::Vector3d &linearVelocity) {
  m_linearVelocity = linearVelocity;
}

const Eigen::Vector3d &RigidBody::getPosition() const {
  return m_position;
}

const Eigen::Vector3d &RigidBody::getLinearVelocity() const {
  return m_linearVelocity;
}

const Mesh &RigidBody::getMesh() const {
  return m_mesh;
}
