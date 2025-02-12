#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include <Eigen/Dense>
#include "graphics/Mesh.h"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

class RigidBody {
public:
	RigidBody(Eigen::Vector3d position,
	          Eigen::Matrix3d massMatrix,
	          const std::vector<Vertex> &vertices,
	          const std::vector<GLuint> &indices,
	          glm::vec3 color);

	glm::vec3 color;

	// Getters
	[[nodiscard]] glm::mat4 getModelMatrix() const;

	[[nodiscard]] const Mesh &getMesh() const;

	[[nodiscard]] const Eigen::Vector3d &getPosition() const;

	[[nodiscard]] const Eigen::Vector3d &getLinearVelocity() const;

	Eigen::Matrix3d &getMassMatrix();

	// Setters
	void setPosition(const Eigen::Vector3d &position);

	void setLinearVelocity(const Eigen::Vector3d &linearVelocity);

private:
	Eigen::Vector3d m_position;
	Eigen::Vector3d m_linearVelocity;
	Eigen::Matrix3d m_massMatrix;

	Mesh m_mesh;
};


#endif // RIGIDBODY_H
