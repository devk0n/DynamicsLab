#ifndef SYSTEM_VISUALIZER_H
#define SYSTEM_VISUALIZER_H

#include "SphericalJoint.h"
#include "RevoluteJoint.h"
#include "Spring.h"
#include "DistanceConstraint.h"
#include "Dynamics.h"
#include "ShaderManager.h"
#include "Logger.h"
#include "UniversalJoint.h"

class SystemVisualizer {
public:
  explicit SystemVisualizer(ShaderManager &shaderManager)
      : m_shaderManager(shaderManager) {
    LOG_DEBUG("SystemVisualizer created");
    initializeCube();
    initializeLine();
    initializeCylinder();
  }

  ~SystemVisualizer() {
    LOG_DEBUG("SystemVisualizer destroyed");
    cleanupCube();
    cleanupCylinder();
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
  GLuint m_lineVAO{}, m_lineVBO{};
  unsigned int m_cubeVAO{}, m_cubeVBO{}, m_cubeEBO{};
  unsigned int m_cylinderVAO{}, m_cylinderVBO{}, m_cylinderEBO{};
  int m_cylinderIndexCount = 0;

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

  void initializeCylinder(int segments = 32) {
    std::vector<float> vertices;
    std::vector<unsigned int> indices;

    const float radius = 0.5f;
    const float halfHeight = 0.5f;
    const float PI = 3.14159265359f;

    // Center points for caps
    vertices.insert(vertices.end(), {0.0f, halfHeight, 0.0f, 0.0f, 1.0f, 0.0f});  // Top center
    vertices.insert(vertices.end(), {0.0f, -halfHeight, 0.0f, 0.0f, -1.0f, 0.0f}); // Bottom center

    for (int i = 0; i <= segments; ++i) {
      float angle = (2.0f * PI * i) / segments;
      float x = std::cos(angle) * radius;
      float z = std::sin(angle) * radius;

      // Top ring
      vertices.insert(vertices.end(), {x, halfHeight, z, 0.0f, 1.0f, 0.0f});

      // Bottom ring
      vertices.insert(vertices.end(), {x, -halfHeight, z, 0.0f, -1.0f, 0.0f});

      // Side top
      vertices.insert(vertices.end(), {x, halfHeight, z, x, 0.0f, z});

      // Side bottom
      vertices.insert(vertices.end(), {x, -halfHeight, z, x, 0.0f, z});
    }

    int offset = 2;
    for (int i = 0; i < segments; ++i) {
      int top = offset + i * 4;
      int bottom = top + 1;
      int topNext = offset + ((i + 1) % segments) * 4;
      int bottomNext = topNext + 1;

      int sideTop = top + 2;
      int sideBottom = top + 3;
      int sideTopNext = topNext + 2;
      int sideBottomNext = topNext + 3;

      indices.push_back(0);
      indices.push_back(top);
      indices.push_back(topNext);

      indices.push_back(1);
      indices.push_back(bottomNext);
      indices.push_back(bottom);

      indices.push_back(sideTop);
      indices.push_back(sideBottom);
      indices.push_back(sideBottomNext);

      indices.push_back(sideTop);
      indices.push_back(sideBottomNext);
      indices.push_back(sideTopNext);

    }

    m_cylinderIndexCount = static_cast<int>(indices.size());

    glGenVertexArrays(1, &m_cylinderVAO);
    glGenBuffers(1, &m_cylinderVBO);
    glGenBuffers(1, &m_cylinderEBO);

    glBindVertexArray(m_cylinderVAO);

    glBindBuffer(GL_ARRAY_BUFFER, m_cylinderVBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_cylinderEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void*>(0));
    glEnableVertexAttribArray(0);

    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), reinterpret_cast<void*>(3 * sizeof(float)));
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
    m_shaderManager.useShader("bodyShader");

    // Set common uniforms
    m_shaderManager.setUniform("lightPos", glm::vec3(200.0f, 400.0f, 100.0f));
    m_shaderManager.setUniform("viewPos", cameraPosition);
    m_shaderManager.setUniform("lightColor", glm::vec3(1.0f, 1.0f, 1.0f));
    m_shaderManager.setUniform("view", viewMatrix);
    m_shaderManager.setUniform("projection", projectionMatrix);

    for (const auto &body : system.getBodies()) {
      m_shaderManager.setUniform("objectColor", body->getColor());

      glm::mat4 modelMatrix(1.0f);
      modelMatrix = translate(modelMatrix, body->getPositionVec3());
      modelMatrix *= glm::mat4_cast(body->getOrientationQuat());
      modelMatrix = scale(modelMatrix, body->getSizeVec3());

      m_shaderManager.setUniform("model", modelMatrix);

      switch (body->getGeometryType()) {
        case Proton::GeometryType::Cube:
          drawCube();
        break;
        case Proton::GeometryType::Cylinder:
          drawCylinder();
        break;
      }
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
                                               glm::length(worldB - worldA));

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
      if (auto *bj = dynamic_cast<Proton::SphericalJoint *>(constraint.get())) {
        processJoint(bj, constraintVertices);
      } else if (auto *dc = dynamic_cast<Proton::DistanceConstraint *>(constraint.get())) {
        processJoint(dc, constraintVertices);
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
      const Proton::RevoluteJoint* rj,
      std::vector<VertexData>& vertices) {
    // Constants
    constexpr float axisScale = 0.5f;
    constexpr glm::vec4 cyanColor(0.0f, 1.0f, 1.0f, 1.0f);
    constexpr glm::vec4 orangeColor(1.0f, 0.5f, 0.0f, 1.0f);
    constexpr glm::vec4 greenColor(0.5f, 1.0f, 0.0f, 1.0f);

    // Get attachment points in world space
    const glm::vec3 pos1 = getWorldAttachmentPoint(rj->getBodyA(), rj->getLocalPointA());
    const glm::vec3 pos2 = getWorldAttachmentPoint(rj->getBodyB(), rj->getLocalPointB());

    // Draw body-to-anchor lines (cyan)
    vertices.push_back(VertexData{rj->getBodyA()->getPositionVec3(), cyanColor});
    vertices.push_back(VertexData{pos1, cyanColor});
    vertices.push_back(VertexData{rj->getBodyB()->getPositionVec3(), cyanColor});
    vertices.push_back(VertexData{pos2, cyanColor});

    // Get world-space rotation axes
    const glm::quat rotA = rj->getBodyA()->getOrientationQuat();
    const glm::quat rotB = rj->getBodyB()->getOrientationQuat();
    const glm::vec3 worldAxisA = rotA * glm::vec3(rj->getAxisA().x(),
                                                 rj->getAxisA().y(),
                                                 rj->getAxisA().z());
    const glm::vec3 worldAxisB = rotB * glm::vec3(rj->getAxisB().x(),
                                                 rj->getAxisB().y(),
                                                 rj->getAxisB().z());

    // Calculate perpendicular axes using the joint's constraint axes
    const glm::vec3 worldConstr1A = rotA * glm::vec3(rj->getAxisM().x(),
                                                    rj->getAxisM().y(),
                                                    rj->getAxisM().z());
    const glm::vec3 worldConstr2A = rotA * glm::vec3(rj->getAxisN().x(),
                                                    rj->getAxisN().y(),
                                                    rj->getAxisN().z());

    // Draw rotation axes (orange)
    auto drawAxis = [&](const glm::vec3& pos, const glm::vec3& axis, const glm::vec4& color) {
        vertices.push_back(VertexData{pos, color});
        vertices.push_back(VertexData{pos + axisScale * axis, color});
        vertices.push_back(VertexData{pos, color});
        vertices.push_back(VertexData{pos - axisScale * axis, color});
    };

    // Draw primary rotation axes
    drawAxis(pos1, worldAxisA, orangeColor);
    drawAxis(pos2, worldAxisB, orangeColor);

    // Draw constraint axes (green) at pos1 (body A's attachment point)
    drawAxis(pos1, worldConstr1A, greenColor);
    drawAxis(pos1, worldConstr2A, greenColor);
  }

  void drawLines(const std::vector<VertexData>& vertices,
                 float lineWidth) const {

    auto vertexCount = static_cast<GLsizei>(vertices.size());

    glBindVertexArray(m_lineVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_lineVBO);

    glBufferData(
      GL_ARRAY_BUFFER,
      static_cast<GLsizeiptr>(vertices.size() * sizeof(VertexData)),
      vertices.data(),
      GL_STATIC_DRAW
    );

    glLineWidth(lineWidth);
    glDrawArrays(GL_LINES, 0, vertexCount);
    glBindVertexArray(0);
  }

  void drawCube() const {
    glBindVertexArray(m_cubeVAO);
    glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
  }

  void drawCylinder() const {
    glBindVertexArray(m_cylinderVAO);
    glDrawElements(GL_TRIANGLES, m_cylinderIndexCount, GL_UNSIGNED_INT, nullptr);
    glBindVertexArray(0);
  }

  void cleanupCylinder() const {
    glDeleteVertexArrays(1, &m_cylinderVAO);
    glDeleteBuffers(1, &m_cylinderVBO);
    glDeleteBuffers(1, &m_cylinderEBO);
  }

};

#endif // SYSTEM_VISUALIZER_H