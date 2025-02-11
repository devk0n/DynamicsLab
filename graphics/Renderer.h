#ifndef RENDERER_H
#define RENDERER_H

#include <glad/glad.h>
#include <vector>
#include <glm/glm.hpp>
#include "Shader.h"
#include "physics/RigidBody.h"
#include "graphics/Mesh.h"

class Renderer {
public:
  bool initialize(const std::string &vertexPath, const std::string &fragmentPath);

  void beginFrame();

  void render(const std::vector<RigidBody> &rigidBodies, const glm::mat4 &view, const glm::mat4 &projection);

  void endFrame();

  void shutdown();

  void setClearColor(float r, float g, float b, float a);

  static void captureScreenshot();

private:
  Shader m_shader;
  float m_clearColor[4] = {0.1f, 0.1f, 0.1f, 1.0f};
  bool m_initialized = false;
};


#endif // RENDERER_H
