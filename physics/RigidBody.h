#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include <Eigen/Dense>
#include "graphics/Mesh.h"

class RigidBody {
public:
    RigidBody(const Mesh& mesh, const Eigen::Vector3f& position, const Eigen::Vector3f& velocity, float mass)
        : m_mesh(mesh), m_position(position), m_velocity(velocity), m_mass(mass) {}

    void update(float deltaTime) {
        // Update position based on velocity
        m_position += m_velocity * deltaTime;
    }

    Eigen::Matrix4f getModelMatrix() const {
        // Create a 4x4 transformation matrix for rendering
        Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
        model.block<3, 1>(0, 3) = m_position; // Set translation
        return model;
    }

    const Mesh& getMesh() const {
        return m_mesh;
    }

    Eigen::Vector3f getPosition() const {
        return m_position;
    }

    void applyForce(const Eigen::Vector3f& force, float deltaTime) {
        // F = ma => a = F / m
        Eigen::Vector3f acceleration = force / m_mass;
        m_velocity += acceleration * deltaTime;
    }

private:
    Mesh m_mesh;
    Eigen::Vector3f m_position; // Position in 3D space
    Eigen::Vector3f m_velocity; // Velocity in 3D space
    float m_mass;               // Mass of the rigid body
};

#endif