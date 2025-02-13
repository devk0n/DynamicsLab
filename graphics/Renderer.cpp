#include "Renderer.h"
#include "ScreenshotUtils.h"
#include "MeshData.h"

// FIXME: Maybe not to good to have these here.
const std::string bodyVertexPath = "assets/shaders/body.vert.glsl";
const std::string bodyFragmentPath = "assets/shaders/body.frag.glsl";

bool Renderer::initialize() {
  // Initialize the shader
  if (!m_bodyShader.loadShader(bodyVertexPath, bodyFragmentPath)) {
    std::cerr << "Failed to initialize body shader" << std::endl;
    return false;
  }

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

void Renderer::render(const std::vector<RigidBody> &rigidBodies, const glm::mat4 &view,
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
    m_bodyShader.setVec3("objectColor", glm::vec3(0.6118, 0.2510, 0.4039));

    body.getMesh().draw(m_bodyShader);
  }
  // Restore default mode to prevent affecting UI elements
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

void Renderer::endFrame() {
  // Post-draw cleanup
}

void Renderer::shutdown() const {
  // Clean up shaders and other resources
  m_bodyShader.cleanup();
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
