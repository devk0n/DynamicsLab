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
  bool initialize();

  void beginFrame();

  void render(const std::vector<RigidBody> &rigidBodies, const glm::mat4 &view, const glm::mat4 &projection);

  void endFrame();

  void shutdown();

  void setClearColor(float r, float g, float b, float a);

  static void captureScreenshot();

  bool wireframeMode = false;

private:
  Shader m_bodyShader;

  float m_clearColor[4] = {0.1f, 0.1f, 0.1f, 1.0f};

  bool m_initialized = false;

};


#endif // RENDERER_H
