#include "Renderer.h"

#include <GroundPoint.h>

#include "ScreenshotUtils.h"
#include "MeshData.h"

// FIXME: Maybe not to good to have these here.
const std::string bodyVertexPath = "assets/shaders/body.vert.glsl";
const std::string bodyFragmentPath = "assets/shaders/body.frag.glsl";

const std::string axesVertexPath = "assets/shaders/axes.vert.glsl";
const std::string axesFragmentPath = "assets/shaders/axes.frag.glsl";

const std::vector<glm::vec3> axesVertices = {
  glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f),
  glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f),
  glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f)
};

bool Renderer::initialize() {
  // Initialize the shader
  if (!m_bodyShader.loadShader(bodyVertexPath, bodyFragmentPath)) {
    std::cerr << "Failed to initialize body shader" << std::endl;
    return false;
  }
  if (!m_axesShader.loadShader(axesVertexPath, axesFragmentPath)) {
    std::cerr << "Failed to initialize axes shader" << std::endl;
    return false;
  }

  glGenVertexArrays(1, &m_axesVAO);
  glGenBuffers(1, &m_axesVBO);

  glBindVertexArray(m_axesVAO);
  glBindBuffer(GL_ARRAY_BUFFER, m_axesVBO);
  glBufferData(GL_ARRAY_BUFFER, axesVertices.size() * sizeof(glm::vec3), axesVertices.data(), GL_STATIC_DRAW);

  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void *) 0);
  glEnableVertexAttribArray(0);

  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindVertexArray(0);

  m_initialized = true;
  return true;
}

void Renderer::beginFrame() const {
  // Tell OpenGL what color to clear the screen with
  glClearColor(m_clearColor[0], m_clearColor[1], m_clearColor[2], m_clearColor[3]);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // Enable depth testing
  glEnable(GL_DEPTH_TEST);
}

void Renderer::render(const std::vector<GroundPoint> &groundPoints,
                      const std::vector<RigidBody> &rigidBodies,
                      const glm::mat4 &view,
                      const glm::mat4 &projection) const {
  if (!m_initialized) {
    std::cerr << "Renderer not initialized" << std::endl;
    return;
  }

  // Enable or disable wireframe mode
  glPolygonMode(GL_FRONT_AND_BACK, wireframeMode ? GL_LINE : GL_FILL);

  m_bodyShader.use();
  m_bodyShader.setMat4("view", view);
  m_bodyShader.setMat4("projection", projection);

  // Set lighting uniforms
  constexpr glm::vec3 lightPos(20.0f, 40.0f, 20.0f);
  constexpr glm::vec3 lightColor(1.0f, 1.0f, 1.0f);
  const auto viewPos = glm::vec3(view[3]); // Extract camera position from view matrix

  m_bodyShader.setVec3("lightPos", lightPos);
  m_bodyShader.setVec3("lightColor", lightColor);
  m_bodyShader.setVec3("viewPos", viewPos);

  for (const auto &body: rigidBodies) {
    glm::mat4 modelMatrix = body.getModelMatrix();
    m_bodyShader.setMat4("model", modelMatrix);
    m_bodyShader.setVec3("objectColor", body.color);

    body.getMesh().draw(m_bodyShader);
    drawAxes(body.getPosition(), view, projection);
  }

  for (const auto &groundPoint: groundPoints) {
    drawAxes(groundPoint.getPosition(), view, projection);
  }

  // Restore default mode to prevent affecting UI elements
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

void Renderer::drawAxes(const Eigen::Vector3d &translation, const glm::mat4 &view, const glm::mat4 &projection) const {
  // Convert Eigen::Vector3d to glm::vec3
  glm::vec3 glmTranslation(translation.x(), translation.y(), translation.z());

  // Create a model matrix from the translation vector
  glm::mat4 modelMatrix = glm::translate(glm::mat4(1.0f), glmTranslation);

  m_axesShader.use();
  m_axesShader.setMat4("model", modelMatrix);
  m_axesShader.setMat4("view", view);
  m_axesShader.setMat4("projection", projection);

  // Bind the VAO
  glBindVertexArray(m_axesVAO);

  // Draw X-axis (Cyan)
  m_axesShader.setVec3("color", glm::vec3(0.0f, 1.0f, 1.0f)); // Cyan
  glDrawArrays(GL_LINES, 0, 2); // First two vertices (X-axis)

  // Draw Y-axis (Magenta)
  m_axesShader.setVec3("color", glm::vec3(1.0f, 0.0f, 1.0f)); // Magenta
  glDrawArrays(GL_LINES, 2, 2); // Next two vertices (Y-axis)

  // Draw Z-axis (Yellow)
  m_axesShader.setVec3("color", glm::vec3(1.0f, 1.0f, 0.0f)); // Yellow
  glDrawArrays(GL_LINES, 4, 2); // Last two vertices (Z-axis)

  // Unbind the VAO
  glBindVertexArray(0);
}

void Renderer::endFrame() {
  // Post-draw cleanup
}

void Renderer::shutdown() const {
  m_bodyShader.cleanup();
  m_axesShader.cleanup();
  glDeleteVertexArrays(1, &m_axesVAO);
  glDeleteBuffers(1, &m_axesVBO);
}

void Renderer::captureScreenshot() {
  saveScreenshot(glfwGetCurrentContext());
}

void Renderer::setClearColor(const float r, const float g, const float b, const float a) {
  m_clearColor[0] = r;
  m_clearColor[1] = g;
  m_clearColor[2] = b;
  m_clearColor[3] = a;
}
