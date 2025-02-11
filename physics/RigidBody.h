#ifndef RIGIDBODY_H
#define RIGIDBODY_H

#include <Eigen/Dense>
#include "graphics/Mesh.h"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>

class RigidBody {
public:
    RigidBody(Eigen::Vector3d  position,
              Eigen::Vector4d  orientation,
              const std::vector<Vertex>& vertices,
              const std::vector<GLuint>& indices);

    [[nodiscard]] glm::mat4 getModelMatrix() const;

    [[nodiscard]] const Mesh& getMesh() const;

    void setPosition(const Eigen::Vector3d& position) {
        m_position = position;
    }

    void setOrientation(const Eigen::Vector4d& orientation) {
        m_orientation = orientation;
    }

    [[nodiscard]] const Eigen::Vector3d& getPosition() const {
        return m_position;
    }

    [[nodiscard]] const Eigen::Vector4d& getOrientation() const {
        return m_orientation;
    }

private:
    Eigen::Vector3d m_position;
    Eigen::Vector4d m_orientation;
    Mesh m_mesh;
};


#endif // RIGIDBODY_H
