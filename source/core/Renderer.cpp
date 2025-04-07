#include "Renderer.h"
#include "PCH.h"

#include "ShaderManager.h"
#include "Camera.h"
#include "Logger.h"

// Vertex data for the grid
static float gridVertices[] = {
  -1000.0f, -1000.0f, 0.0f,
   1000.0f, -1000.0f, 0.0f,
  -1000.0f,  1000.0f, 0.0f,
   1000.0f,  1000.0f, 0.0f
};

// Full-screen quad vertices
float skyVertices[] = {
  -1.0f,  1.0f, 0.0f,
  -1.0f, -1.0f, 0.0f,
   1.0f,  1.0f, 0.0f,
   1.0f, -1.0f, 0.0f
};

Renderer::Renderer()
  : m_shaderManager(std::make_unique<ShaderManager>()),
    m_gridVAO(0),
    m_gridVBO(0) {
}

Renderer::~Renderer() {
  glDeleteVertexArrays(1, &m_gridVAO);
  glDeleteBuffers(1, &m_gridVBO);
  LOG_INFO("Renderer destroyed");
}

bool Renderer::initialize() {
  // Setup global OpenGL state.
  glEnable(GL_DEPTH_TEST);
  glDisable(GL_CULL_FACE);
  glCullFace(GL_BACK);
  glFrontFace(GL_CCW);
  glEnable(GL_MULTISAMPLE);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  // Load common shaders.
  if (!m_shaderManager->loadShader("grid", "../assets/shaders/grid.vert",
                                  "../assets/shaders/grid.frag")) {
    LOG_ERROR("Failed to load grid shader");
    return false;
                                  }
  if (!m_shaderManager->loadShader("sky", "../assets/shaders/sky.vert",
                                  "../assets/shaders/sky.frag")) {
    LOG_ERROR("Failed to load sky shader");
    return false;
                                  }

  // Load the line shader in Renderer::initialize()
  if (!m_shaderManager->loadShader("line", "../assets/shaders/line.vert",
                                  "../assets/shaders/line.frag")) {
    LOG_ERROR("Failed to load line shader");
    return false;
                                  }

  // Initialize resources for grid and sky.
  initGrid();

  LOG_INFO("Renderer initialized successfully!");
  return true;
}

void Renderer::beginFrame() {
  clearScreen();
  glEnable(GL_DEPTH_TEST);
  // Additional per-frame setup can be done here.
}

void Renderer::clearScreen() {
  glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Renderer::endFrame() {
  // Optionally handle post-processing or other tasks before buffer swap.
}

ShaderManager &Renderer::getShaderManager() const { return *m_shaderManager; }

void Renderer::drawGrid(const Camera &camera) const {
  // Use the grid shader.
  const unsigned int shader = m_shaderManager->getShader("grid");
  glUseProgram(shader);

  // Compute the view-projection matrix.
  const glm::mat4 view = camera.getViewMatrix();
  const glm::mat4 projection = camera.getProjectionMatrix();
  glm::mat4 viewProjection = projection * view;
  glUniformMatrix4fv(glGetUniformLocation(shader, "u_viewProjection"), 1,
                     GL_FALSE, value_ptr(viewProjection));

  // Inside Renderer::drawGrid(), after setting the viewProjection uniform
  glUniform3f(glGetUniformLocation(shader, "u_fogColor"), 0.1f, 0.1f, 0.1f);
  glUniform1f(glGetUniformLocation(shader, "u_fogStart"), 50.0f);
  glUniform1f(glGetUniformLocation(shader, "u_fogEnd"), 500.0f);

  // Draw the grid.
  glBindVertexArray(m_gridVAO);
  glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
  glBindVertexArray(0);
}

bool Renderer::initGrid() {
  glGenVertexArrays(1, &m_gridVAO);
  glBindVertexArray(m_gridVAO);

  glGenBuffers(1, &m_gridVBO);
  glBindBuffer(GL_ARRAY_BUFFER, m_gridVBO);
  glBufferData(GL_ARRAY_BUFFER, sizeof(gridVertices), gridVertices,
               GL_STATIC_DRAW);

  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float),
                        reinterpret_cast<void *>(0));
  glEnableVertexAttribArray(0);

  glBindVertexArray(0);
  return true;
}
