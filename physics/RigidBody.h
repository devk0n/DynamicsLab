#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include <Eigen/Dense>
#include "graphics/Mesh.h"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

class RigidBody {
public:
	RigidBody(Eigen::Vector3d initialPosition,
	          Eigen::Matrix3d massMatrix,
	          const std::vector<Vertex> &vertices,
	          const std::vector<GLuint> &indices,
	          glm::vec3 color,
	          Eigen::Vector3d initialLinearVelocity = Eigen::Vector3d::Zero());

	glm::vec3 color;

	// Getters
	[[nodiscard]] glm::mat4 getModelMatrix() const;

	void reset();

	[[nodiscard]] const Mesh &getMesh() const;

	[[nodiscard]] const Eigen::Vector3d &getPosition() const;

	[[nodiscard]] const Eigen::Vector3d &getLinearVelocity() const;

	Eigen::Matrix3d &getMassMatrix();

	double getMass() const;

	// Setters
	void setPosition(const Eigen::Vector3d &position);

	void setLinearVelocity(const Eigen::Vector3d &linearVelocity);

private:
	Eigen::Vector3d m_position;
	Eigen::Vector3d m_linearVelocity;

	Eigen::Matrix3d m_massMatrix;

	Mesh m_mesh;

	const Eigen::Vector3d m_initialPosition;
	const Eigen::Vector3d m_initialLinearVelocity;
};


#endif // RIGIDBODY_H
