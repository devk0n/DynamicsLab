#ifndef SYSTEM_VISUALIZER_H
#define SYSTEM_VISUALIZER_H

#include <glm/glm.hpp>

#include "DistanceConstraint.h"
#include "BallJoint.h"
#include "Dynamics.h"
#include "ShaderManager.h"
#include "Logger.h"

class SystemVisualizer {
public:
  explicit SystemVisualizer(ShaderManager &shaderManager)
    : m_shaderManager(shaderManager) {
    LOG_DEBUG("SystemVisualizer created");
    // Initialize the cube mesh (vertices, VAO, VBO, etc.)
    initializeCube();
    initializeLine();
  }

  ~SystemVisualizer() {
    LOG_DEBUG("SystemVisualizer destroyed");
    // Clean up the cube mesh (delete VAO, VBO, etc.)
    cleanupCube();
  }

  void render(
    const Proton::Dynamics &system,
    const glm::vec3 &cameraPosition,
    const glm::mat4 &viewMatrix,
    const glm::mat4 &projectionMatrix) const {
    m_shaderManager.useShader("cubeShader");

    // Set light and material properties
    m_shaderManager.setUniform("lightPos", glm::vec3(200.0f, 400.0f, 100.0f));
    // Light position
    m_shaderManager.setUniform("viewPos", cameraPosition); // Camera position
    m_shaderManager.setUniform("lightColor", glm::vec3(1.0f, 1.0f, 1.0f));
    // White light

    // Set transformation matrices
    m_shaderManager.setUniform("view", viewMatrix);
    m_shaderManager.setUniform("projection", projectionMatrix);

    // Render each body
    for (const auto& body: system.getBodies()) {
      auto modelMatrix = glm::mat4(1.0f); // Start with an identity matrix

      // Apply translation
      modelMatrix = translate(modelMatrix, body->getPositionVec3());

      // Apply orientation (rotation)
      glm::quat orientation = body->getOrientationQuat();
      modelMatrix = modelMatrix * mat4_cast(orientation);
      // Convert quaternion to matrix and apply

      m_shaderManager.setUniform("objectColor", glm::vec4(0.98, 0.98, 0.96, 1.0));
      // Set color (white)
      m_shaderManager.setUniform("model", modelMatrix); // Set model matrix
      drawCube();                                       // Render the cube
    }

    // Render constraints
    m_shaderManager.useShader("lineShader");
    m_shaderManager.setUniform("view", viewMatrix);
    m_shaderManager.setUniform("projection", projectionMatrix);
    m_shaderManager.setUniform("model", glm::mat4(1.0f)); // Identity matrix
    m_shaderManager.setUniform("lineColor", glm::vec4(1.0f, 1.0f, 0.0f, 1.0f));

    std::vector<glm::vec3> linePoints;
    for (const auto& constraint : system.getConstraints()) {
      if (auto* dc = dynamic_cast<Proton::DistanceConstraint*>(constraint.get())) {
        // Distance constraint: line between body positions
        auto pos1 = dc->getBody1()->getPositionVec3();
        auto pos2 = dc->getBody2()->getPositionVec3();
        linePoints.push_back(pos1);
        linePoints.push_back(pos2);
      } else if (auto* sj = dynamic_cast<Proton::BallJoint*>(constraint.get())) {
        // Get local attachment points
        Eigen::Vector3d local1 = sj->getLocal1();
        Eigen::Vector3d local2 = sj->getLocal2();

        // Convert Eigen vectors to glm
        glm::vec3 glmLocal1(local1.x(), local1.y(), local1.z());
        glm::vec3 glmLocal2(local2.x(), local2.y(), local2.z());

        // Get body positions and orientations
        glm::vec3 body1Pos = sj->getBody1()->getPositionVec3();
        glm::quat body1Rot = sj->getBody1()->getOrientationQuat();
        glm::vec3 body2Pos = sj->getBody2()->getPositionVec3();
        glm::quat body2Rot = sj->getBody2()->getOrientationQuat();

        // Calculate world positions of attachment points
        glm::vec3 pos1 = body1Pos + body1Rot * glmLocal1;
        glm::vec3 pos2 = body2Pos + body2Rot * glmLocal2;

        // Connection line between attachment points
        // linePoints.push_back(pos1);
        // linePoints.push_back(pos2);

        // Local vector for body 1 (from body origin to attachment point)
        linePoints.push_back(body1Pos);
        linePoints.push_back(pos1);

        // Local vector for body 2 (from body origin to attachment point)
        linePoints.push_back(body2Pos);
        linePoints.push_back(pos2);
      }
    }

    // Draw lines
    glBindVertexArray(m_lineVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_lineVBO);
    glBufferData(GL_ARRAY_BUFFER, linePoints.size() * sizeof(glm::vec3),
                 linePoints.data(), GL_DYNAMIC_DRAW);
    glLineWidth(2.0f);
    glDrawArrays(GL_LINES, 0, linePoints.size());
    glBindVertexArray(0);
  }

private:
  ShaderManager &m_shaderManager;

  unsigned int m_cubeVAO{}, m_cubeVBO{}, m_cubeEBO{};

  GLuint m_lineVAO{}, m_lineVBO{};

  void initializeLine() {
    glGenVertexArrays(1, &m_lineVAO);
    glGenBuffers(1, &m_lineVBO);

    glBindVertexArray(m_lineVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_lineVBO);

    // Initially allocate buffer (size will be updated dynamically)
    glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float),
                          static_cast<void *>(nullptr));
    glEnableVertexAttribArray(0);

    glBindVertexArray(0);
  }

  void initializeCube() {
    // Define the vertices of a cube
    constexpr float vertices[] = {
      // Positions          // Normals
      // Back face
      -0.5f, -0.5f, -0.5f, 0.0f, 0.0f, -1.0f, // Bottom-left
      0.5f, -0.5f, -0.5f, 0.0f, 0.0f, -1.0f,  // Bottom-right
      0.5f, 0.5f, -0.5f, 0.0f, 0.0f, -1.0f,   // Top-right
      -0.5f, 0.5f, -0.5f, 0.0f, 0.0f, -1.0f,  // Top-left

      // Front face
      -0.5f, -0.5f, 0.5f, 0.0f, 0.0f, 1.0f, // Bottom-left
      0.5f, -0.5f, 0.5f, 0.0f, 0.0f, 1.0f,  // Bottom-right
      0.5f, 0.5f, 0.5f, 0.0f, 0.0f, 1.0f,   // Top-right
      -0.5f, 0.5f, 0.5f, 0.0f, 0.0f, 1.0f,  // Top-left

      // Left face
      -0.5f, 0.5f, 0.5f, -1.0f, 0.0f, 0.0f,   // Top-right
      -0.5f, 0.5f, -0.5f, -1.0f, 0.0f, 0.0f,  // Top-left
      -0.5f, -0.5f, -0.5f, -1.0f, 0.0f, 0.0f, // Bottom-left
      -0.5f, -0.5f, 0.5f, -1.0f, 0.0f, 0.0f,  // Bottom-right

      // Right face
      0.5f, 0.5f, 0.5f, 1.0f, 0.0f, 0.0f,   // Top-left
      0.5f, 0.5f, -0.5f, 1.0f, 0.0f, 0.0f,  // Top-right
      0.5f, -0.5f, -0.5f, 1.0f, 0.0f, 0.0f, // Bottom-right
      0.5f, -0.5f, 0.5f, 1.0f, 0.0f, 0.0f,  // Bottom-left

      // Bottom face
      -0.5f, -0.5f, -0.5f, 0.0f, -1.0f, 0.0f, // Bottom-right
      0.5f, -0.5f, -0.5f, 0.0f, -1.0f, 0.0f,  // Bottom-left
      0.5f, -0.5f, 0.5f, 0.0f, -1.0f, 0.0f,   // Top-left
      -0.5f, -0.5f, 0.5f, 0.0f, -1.0f, 0.0f,  // Top-right

      // Top face
      -0.5f, 0.5f, -0.5f, 0.0f, 1.0f, 0.0f, // Top-left
      0.5f, 0.5f, -0.5f, 0.0f, 1.0f, 0.0f,  // Top-right
      0.5f, 0.5f, 0.5f, 0.0f, 1.0f, 0.0f,   // Bottom-right
      -0.5f, 0.5f, 0.5f, 0.0f, 1.0f, 0.0f   // Bottom-left
    };

    // Define the indices for the cube
    const unsigned int indices[] = {
      // Back face
      0, 1, 2, 2, 3, 0,
      // Front face
      4, 5, 6, 6, 7, 4,
      // Left face
      8, 9, 10, 10, 11, 8,
      // Right face
      12, 13, 14, 14, 15, 12,
      // Bottom face
      16, 17, 18, 18, 19, 16,
      // Top face
      20, 21, 22, 22, 23, 20
    };

    // Generate and bind the VAO and VBO
    glGenVertexArrays(1, &m_cubeVAO);
    glGenBuffers(1, &m_cubeVBO);
    glGenBuffers(1, &m_cubeEBO);

    glBindVertexArray(m_cubeVAO);

    // Bind vertex data
    glBindBuffer(GL_ARRAY_BUFFER, m_cubeVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    // Bind index data
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_cubeEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices,
                 GL_STATIC_DRAW);

    // Position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                          static_cast<void *>(nullptr));
    glEnableVertexAttribArray(0);

    // Normal attribute
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                          reinterpret_cast<void *>(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);
  }

  void cleanupCube() const {
    glDeleteVertexArrays(1, &m_cubeVAO);
    glDeleteBuffers(1, &m_cubeVBO);
    glDeleteBuffers(1, &m_cubeEBO);
  }

  void drawCube() const {
    glBindVertexArray(m_cubeVAO);
    glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
  }
};

#endif // SYSTEM_VISUALIZER_H
