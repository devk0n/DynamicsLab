#ifndef SYSTEM_VISUALIZER_H
#define SYSTEM_VISUALIZER_H

#include <RevoluteJoint.h>#include <glm/glm.hpp>
#include "Spring.h"
#include "DistanceConstraint.h"
#include "BallJoint.h"
#include "Dynamics.h"
#include "ShaderManager.h"
#include "Logger.h"
#include "SphericalJoint.h"
#include "UniversalJoint.h"

class SystemVisualizer {
public:
  explicit SystemVisualizer(ShaderManager &shaderManager)
      : m_shaderManager(shaderManager) {
    LOG_DEBUG("SystemVisualizer created");
    initializeCube();
    initializeLine();
  }

  ~SystemVisualizer() {
    LOG_DEBUG("SystemVisualizer destroyed");
    cleanupCube();
  }

  void render(
      const Proton::Dynamics &system,
      const glm::vec3 &cameraPosition,
      const glm::mat4 &viewMatrix,
      const glm::mat4 &projectionMatrix) const {
    renderBodies(system, cameraPosition, viewMatrix, projectionMatrix);
    renderSprings(system, viewMatrix, projectionMatrix);
    renderConstraints(system, viewMatrix, projectionMatrix);
  }

private:
  ShaderManager &m_shaderManager;
  unsigned int m_cubeVAO{}, m_cubeVBO{}, m_cubeEBO{};
  GLuint m_lineVAO{}, m_lineVBO{};

  // Helper structs
  struct VertexData {
    glm::vec3 position;
    glm::vec4 color;
  };

  // Initialization methods
  void initializeLine() {
    glGenVertexArrays(1, &m_lineVAO);
    glGenBuffers(1, &m_lineVBO);

    glBindVertexArray(m_lineVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_lineVBO);
    glBufferData(GL_ARRAY_BUFFER, 0, nullptr, GL_DYNAMIC_DRAW);

    // Position attribute (3 floats)
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(VertexData), static_cast<void *>(nullptr));
    glEnableVertexAttribArray(0);

    // Color attribute (4 floats)
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(VertexData), reinterpret_cast<void *>(offsetof(VertexData, color)));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);
  }

  void initializeCube() {
    constexpr float vertices[] = {
      // Positions            // Normals
      // Back face
      -0.5f, -0.5f, -0.5f,     0.0f, 0.0f, -1.0f,
       0.5f, -0.5f, -0.5f,     0.0f, 0.0f, -1.0f,
       0.5f,  0.5f, -0.5f,     0.0f, 0.0f, -1.0f,
      -0.5f,  0.5f, -0.5f,     0.0f, 0.0f, -1.0f,

      // Front face
      -0.5f, -0.5f,  0.5f,     0.0f, 0.0f, 1.0f,
       0.5f, -0.5f,  0.5f,     0.0f, 0.0f, 1.0f,
       0.5f,  0.5f,  0.5f,     0.0f, 0.0f, 1.0f,
      -0.5f,  0.5f,  0.5f,     0.0f, 0.0f, 1.0f,

      // Left face
      -0.5f,  0.5f,  0.5f,    -1.0f, 0.0f, 0.0f,
      -0.5f,  0.5f, -0.5f,    -1.0f, 0.0f, 0.0f,
      -0.5f, -0.5f, -0.5f,    -1.0f, 0.0f, 0.0f,
      -0.5f, -0.5f,  0.5f,    -1.0f, 0.0f, 0.0f,

      // Right face
       0.5f,  0.5f,  0.5f,     1.0f, 0.0f, 0.0f,
       0.5f,  0.5f, -0.5f,     1.0f, 0.0f, 0.0f,
       0.5f, -0.5f, -0.5f,     1.0f, 0.0f, 0.0f,
       0.5f, -0.5f,  0.5f,     1.0f, 0.0f, 0.0f,

      // Bottom face
      -0.5f, -0.5f, -0.5f,     0.0f, -1.0f, 0.0f,
       0.5f, -0.5f, -0.5f,     0.0f, -1.0f, 0.0f,
       0.5f, -0.5f,  0.5f,     0.0f, -1.0f, 0.0f,
      -0.5f, -0.5f,  0.5f,     0.0f, -1.0f, 0.0f,

      // Top face
      -0.5f,  0.5f, -0.5f,     0.0f, 1.0f, 0.0f,
       0.5f,  0.5f, -0.5f,     0.0f, 1.0f, 0.0f,
       0.5f,  0.5f,  0.5f,     0.0f, 1.0f, 0.0f,
      -0.5f,  0.5f,  0.5f,     0.0f, 1.0f, 0.0f
    };

    const unsigned int indices[] = {
      0,  1,  2,  2,  3,  0, // Back face
      4,  5,  6,  6,  7,  4, // Front face
      8,  9, 10, 10, 11,  8, // Left face
     12, 13, 14, 14, 15, 12, // Right face
     16, 17, 18, 18, 19, 16, // Bottom face
     20, 21, 22, 22, 23, 20  // Top face
    };

    glGenVertexArrays(1, &m_cubeVAO);
    glGenBuffers(1, &m_cubeVBO);
    glGenBuffers(1, &m_cubeEBO);

    glBindVertexArray(m_cubeVAO);

    glBindBuffer(GL_ARRAY_BUFFER, m_cubeVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_cubeEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices,
                 GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                          static_cast<void *>(nullptr));
    glEnableVertexAttribArray(0);

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

  // Rendering methods
  void renderBodies(const Proton::Dynamics &system,
                    const glm::vec3 &cameraPosition,
                    const glm::mat4 &viewMatrix,
                    const glm::mat4 &projectionMatrix) const {
    m_shaderManager.useShader("cubeShader");

    // Set common uniforms
    m_shaderManager.setUniform("lightPos", glm::vec3(200.0f, 400.0f, 100.0f));
    m_shaderManager.setUniform("viewPos", cameraPosition);
    m_shaderManager.setUniform("lightColor", glm::vec3(1.0f, 1.0f, 1.0f));
    m_shaderManager.setUniform("view", viewMatrix);
    m_shaderManager.setUniform("projection", projectionMatrix);
    m_shaderManager.setUniform("objectColor",
                               glm::vec4(0.6118, 0.2510, 0.4039, 1.0));

    for (const auto &body: system.getBodies()) {
      glm::mat4 modelMatrix(1.0f);
      modelMatrix = translate(modelMatrix, body->getPositionVec3());
      modelMatrix *= mat4_cast(body->getOrientationQuat());
      modelMatrix = scale(modelMatrix, body->getSizeVec3());

      m_shaderManager.setUniform("model", modelMatrix);
      drawCube();
    }
  }

  void renderSprings(const Proton::Dynamics &system,
                     const glm::mat4 &viewMatrix,
                     const glm::mat4 &projectionMatrix) const {
    m_shaderManager.useShader("lineShader");
    m_shaderManager.setUniform("view", viewMatrix);
    m_shaderManager.setUniform("projection", projectionMatrix);
    m_shaderManager.setUniform("model", glm::mat4(1.0f));

    std::vector<VertexData> springVertices;

    for (const auto &forceGen: system.getForceElements()) {
      if (auto *spring = dynamic_cast<Proton::Spring *>(forceGen.get())) {
        // Get world positions of attachment points
        glm::vec3 worldA = getWorldAttachmentPoint(
          spring->getBodyA(), spring->getLocalPointA());
        glm::vec3 worldB = getWorldAttachmentPoint(
          spring->getBodyB(), spring->getLocalPointB());

        glm::vec4 color = calculateSpringColor(static_cast<float>(spring->getRestLength()),
                                               length(worldB - worldA));

        // Add both points to the buffer
        springVertices.push_back({worldA, color});
        springVertices.push_back({worldB, color});

        // Add visualization of local points (optional)
        springVertices.push_back({
          spring->getBodyA()->getPositionVec3(), {1.0f, 0.0f, 1.0f, 1.0f}
        });
        springVertices.push_back({worldA, {1.0f, 0.0f, 1.0f, 1.0f}});
        springVertices.push_back({
          spring->getBodyB()->getPositionVec3(), {1.0f, 0.0f, 1.0f, 1.0f}
        });
        springVertices.push_back({worldB, {1.0f, 0.0f, 1.0f, 1.0f}});
      }
    }

    if (!springVertices.empty()) {
      drawLines(springVertices, 3.0f);
    }
  }

  void renderConstraints(const Proton::Dynamics &system,
                         const glm::mat4 &viewMatrix,
                         const glm::mat4 &projectionMatrix) const {
    m_shaderManager.useShader("lineShader");
    m_shaderManager.setUniform("view", viewMatrix);
    m_shaderManager.setUniform("projection", projectionMatrix);
    m_shaderManager.setUniform("model", glm::mat4(1.0f));

    std::vector<VertexData> constraintVertices;

    for (const auto &constraint: system.getConstraints()) {
      if (auto *dc = dynamic_cast<Proton::DistanceConstraint *>(constraint.
        get())) {
        // Distance constraint: line between body positions
        constraintVertices.push_back({
          dc->getBodyA()->getPositionVec3(), {1.0f, 1.0f, 0.0f, 1.0f}
        });
        constraintVertices.push_back({
          dc->getBodyB()->getPositionVec3(), {1.0f, 1.0f, 0.0f, 1.0f}
        });
      } else if (auto *bj = dynamic_cast<Proton::BallJoint *>(constraint.get())) {
        processJoint(bj, constraintVertices);
      } else if (auto *sj = dynamic_cast<Proton::SphericalJoint *>(constraint.get())) {
        processJoint(sj, constraintVertices);
      } else if (auto *uj = dynamic_cast<Proton::UniversalJoint *>(constraint.get())) {
        processUniversalJoint(uj, constraintVertices);
      } else if (auto *rj = dynamic_cast<Proton::RevoluteJoint *>(constraint.get())) {
        processRevoluteJoint(rj, constraintVertices);
      }
    }

    if (!constraintVertices.empty()) {
      drawLines(constraintVertices, 2.0f);
    }
  }

  // Helper methods
  static glm::vec3 getWorldAttachmentPoint(
      const Proton::Body *body,
      const Eigen::Vector3d &localPoint) {
    return body->getPositionVec3() +
           body->getOrientationQuat() *
           glm::vec3(localPoint.x(), localPoint.y(), localPoint.z());
  }

  [[nodiscard]] static glm::vec4 calculateSpringColor(float restLength, float currentLength) {
    float ratio = currentLength / restLength;

    if (ratio < 1.0f) {
      float t = glm::clamp((ratio - 0.7f) / 0.3f, 0.0f, 1.0f);
      return glm::mix(glm::vec4(1.0f, 0.0f, 0.0f, 1.0f), // Red
                      glm::vec4(1.0f, 1.0f, 0.0f, 1.0f), // Yellow
                      t);
    }

    float t = glm::clamp((ratio - 1.0f) / 0.3f, 0.0f, 1.0f);
    return glm::mix(glm::vec4(1.0f, 1.0f, 0.0f, 1.0f), // Yellow
                    glm::vec4(0.0f, 1.0f, 0.0f, 1.0f), // Green
                    t);
  }

  template<typename JointType>
  static void processJoint(JointType *joint, std::vector<VertexData> &vertices) {
    glm::vec3 pos1 = getWorldAttachmentPoint(joint->getBodyA(),
                                             joint->getLocalPointA());
    glm::vec3 pos2 = getWorldAttachmentPoint(joint->getBodyB(),
                                             joint->getLocalPointB());

    // Connection line between attachment points
    vertices.push_back({pos1, {1.0f, 1.0f, 0.0f, 1.0f}});
    vertices.push_back({pos2, {1.0f, 1.0f, 0.0f, 1.0f}});

    // Local vectors from body origin to attachment point
    vertices.push_back({
      joint->getBodyA()->getPositionVec3(), {0.0f, 1.0f, 1.0f, 1.0f}
    });
    vertices.push_back({pos1, {0.0f, 1.0f, 1.0f, 1.0f}});
    vertices.push_back({
      joint->getBodyB()->getPositionVec3(), {0.0f, 1.0f, 1.0f, 1.0f}
    });
    vertices.push_back({pos2, {0.0f, 1.0f, 1.0f, 1.0f}});
  }

  static void processUniversalJoint(
      const Proton::UniversalJoint *uj,
      std::vector<VertexData> &vertices) {
    glm::vec3 pos1 = getWorldAttachmentPoint(uj->getBodyA(),
                                             uj->getLocalPointA());
    glm::vec3 pos2 = getWorldAttachmentPoint(uj->getBodyB(),
                                             uj->getLocalPointB());

    // Local vectors
    vertices.push_back({
      uj->getBodyA()->getPositionVec3(), {0.0f, 1.0f, 1.0f, 1.0f}
    });
    vertices.push_back({pos1, {0.0f, 1.0f, 1.0f, 1.0f}});
    vertices.push_back({
      uj->getBodyB()->getPositionVec3(), {0.0f, 1.0f, 1.0f, 1.0f}
    });
    vertices.push_back({pos2, {0.0f, 1.0f, 1.0f, 1.0f}});

    // Draw the axis lines
    constexpr float axisScale = 0.5f;
    glm::vec3 worldAxis1 = uj->getBodyA()->getOrientationQuat() *
                           glm::vec3(uj->getAxisA().x(), uj->getAxisA().y(),
                                     uj->getAxisA().z());
    glm::vec3 worldAxis2 = uj->getBodyB()->getOrientationQuat() *
                           glm::vec3(uj->getAxisB().x(), uj->getAxisB().y(),
                                     uj->getAxisB().z());

    // Axis lines
    vertices.push_back({pos1, {1.0f, 0.5f, 0.0f, 1.0f}});
    vertices.push_back(
      {pos1 + axisScale * worldAxis1, {1.0f, 0.5f, 0.0f, 1.0f}});
    vertices.push_back({pos1, {1.0f, 0.5f, 0.0f, 1.0f}});
    vertices.push_back(
      {pos1 - axisScale * worldAxis1, {1.0f, 0.5f, 0.0f, 1.0f}});
    vertices.push_back({pos2, {1.0f, 0.5f, 0.0f, 1.0f}});
    vertices.push_back(
      {pos2 + axisScale * worldAxis2, {1.0f, 0.5f, 0.0f, 1.0f}});
    vertices.push_back({pos2, {1.0f, 0.5f, 0.0f, 1.0f}});
    vertices.push_back(
      {pos2 - axisScale * worldAxis2, {1.0f, 0.5f, 0.0f, 1.0f}});
  }

  static void processRevoluteJoint(
    const Proton::RevoluteJoint *rj,
    std::vector<VertexData> &vertices) {
    glm::vec3 pos1 = getWorldAttachmentPoint(rj->getBodyA(),
                                           rj->getLocalPointA());
    glm::vec3 pos2 = getWorldAttachmentPoint(rj->getBodyB(),
                                           rj->getLocalPointB());

    // Local vectors
    vertices.push_back({
        rj->getBodyA()->getPositionVec3(), {0.0f, 1.0f, 1.0f, 1.0f}
    });
    vertices.push_back({pos1, {0.0f, 1.0f, 1.0f, 1.0f}});
    vertices.push_back({
        rj->getBodyB()->getPositionVec3(), {0.0f, 1.0f, 1.0f, 1.0f}
    });
    vertices.push_back({pos2, {0.0f, 1.0f, 1.0f, 1.0f}});

    // Draw the axis lines
    constexpr float axisScale = 0.5f;
    glm::vec3 worldAxis1 = rj->getBodyA()->getOrientationQuat() *
                         glm::vec3(rj->getAxisA().x(), rj->getAxisA().y(),
                                 rj->getAxisA().z());
    glm::vec3 worldAxis2 = rj->getBodyB()->getOrientationQuat() *
                         glm::vec3(rj->getAxisB().x(), rj->getAxisB().y(),
                                 rj->getAxisB().z());

    // Calculate the perpendicular axis (last axis of the frame)
    glm::vec3 perpendicularAxis1 = cross(worldAxis1, glm::vec3(1.0f, 0.0f, 0.0f));
    if (length(perpendicularAxis1) < 0.001f) { // If parallel to X-axis
        perpendicularAxis1 = cross(worldAxis1, glm::vec3(0.0f, 1.0f, 0.0f));
    }
    perpendicularAxis1 = normalize(perpendicularAxis1);

    glm::vec3 perpendicularAxis2 = cross(worldAxis2, glm::vec3(1.0f, 0.0f, 0.0f));
    if (length(perpendicularAxis2) < 0.001f) { // If parallel to X-axis
        perpendicularAxis2 = cross(worldAxis2, glm::vec3(0.0f, 1.0f, 0.0f));
    }
    perpendicularAxis2 = normalize(perpendicularAxis2);

    // Primary axis (original rotation axis)
    vertices.push_back({pos1, {1.0f, 0.5f, 0.0f, 1.0f}}); // Orange
    vertices.push_back({pos1 + axisScale * worldAxis1, {1.0f, 0.5f, 0.0f, 1.0f}});
    vertices.push_back({pos1, {1.0f, 0.5f, 0.0f, 1.0f}});
    vertices.push_back({pos1 - axisScale * worldAxis1, {1.0f, 0.5f, 0.0f, 1.0f}});

    vertices.push_back({pos2, {1.0f, 0.5f, 0.0f, 1.0f}});
    vertices.push_back({pos2 + axisScale * worldAxis2, {1.0f, 0.5f, 0.0f, 1.0f}});
    vertices.push_back({pos2, {1.0f, 0.5f, 0.0f, 1.0f}});
    vertices.push_back({pos2 - axisScale * worldAxis2, {1.0f, 0.5f, 0.0f, 1.0f}});

    // Perpendicular axis (last axis of the frame)
    vertices.push_back({pos1, {0.5f, 1.0f, 0.0f, 1.0f}}); // Green
    vertices.push_back({pos1 + axisScale * perpendicularAxis1, {0.5f, 1.0f, 0.0f, 1.0f}});
    vertices.push_back({pos1, {0.5f, 1.0f, 0.0f, 1.0f}});
    vertices.push_back({pos1 - axisScale * perpendicularAxis1, {0.5f, 1.0f, 0.0f, 1.0f}});

    vertices.push_back({pos2, {0.5f, 1.0f, 0.0f, 1.0f}});
    vertices.push_back({pos2 + axisScale * perpendicularAxis2, {0.5f, 1.0f, 0.0f, 1.0f}});
    vertices.push_back({pos2, {0.5f, 1.0f, 0.0f, 1.0f}});
    vertices.push_back({pos2 - axisScale * perpendicularAxis2, {0.5f, 1.0f, 0.0f, 1.0f}});
  }

  void drawLines(const std::vector<VertexData> &vertices,
                 float lineWidth) const {
    glBindVertexArray(m_lineVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_lineVBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(VertexData),
                 vertices.data(), GL_DYNAMIC_DRAW);
    glLineWidth(lineWidth);
    glDrawArrays(GL_LINES, 0, vertices.size());
    glBindVertexArray(0);
  }

  void drawCube() const {
    glBindVertexArray(m_cubeVAO);
    glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
  }
};

#endif // SYSTEM_VISUALIZER_H
