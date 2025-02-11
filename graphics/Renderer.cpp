#include "Renderer.h"

bool Renderer::initialize(const std::string &vertexPath, const std::string &fragmentPath) {
  // Initialize the shader
  if (!m_shader.loadShader(vertexPath, fragmentPath)) {
    std::cerr << "Failed to initialize shader" << std::endl;
    return false;
  }
  
  m_initialized = true;
  return true;
}

void Renderer::setClearColor(float r, float g, float b, float a) {
  m_clearColor[0] = r;
  m_clearColor[1] = g;
  m_clearColor[2] = b;
  m_clearColor[3] = a;
}

void Renderer::beginFrame() {
  // Tell OpenGL what color to clear the screen with
  glClearColor(m_clearColor[0], m_clearColor[1], m_clearColor[2], m_clearColor[3]);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // Enable depth testing
  glEnable(GL_DEPTH_TEST);
}

void Renderer::render(const std::vector<RigidBody> &rigidBodies, const glm::mat4 &view, const glm::mat4 &projection) {
  if (!m_initialized) {
    std::cerr << "Renderer not initialized" << std::endl;
    return;
  }

  m_shader.use();
  m_shader.setMat4("view", view);
  m_shader.setMat4("projection", projection);

  // Set lighting uniforms
  glm::vec3 lightPos(20.0f, 40.0f, 20.0f);
  glm::vec3 lightColor(1.0f, 1.0f, 1.0f);
  auto viewPos = glm::vec3(view[3]);  // Extract camera position from view matrix

  m_shader.setVec3("lightPos", lightPos);
  m_shader.setVec3("lightColor", lightColor);
  m_shader.setVec3("viewPos", viewPos);

  for (const auto &body: rigidBodies) {
    m_shader.setMat4("model", body.getModelMatrix());
    m_shader.setVec3("objectColor", glm::vec3(0.2, 0.7, 0.4));  // Assuming each body has a color

    body.getMesh().draw(m_shader);
  }
}


void Renderer::endFrame() {
  // Post-draw cleanup
}

void Renderer::shutdown() {
  // Clean up shaders and other resources
  // m_shader.cleanup();
}

