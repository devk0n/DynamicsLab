#ifndef RENDERER_H
#define RENDERER_H

#include <vector>
#include <glm/glm.hpp>

#include "Shader.h"
#include "physics/RigidBody.h"

class Renderer {
public:
  bool initialize();

  void beginFrame() const;

  void render(const std::vector<RigidBody> &rigidBodies, const glm::mat4 &view, const glm::mat4 &projection) const;

  static void endFrame();

  void shutdown() const;

  void setClearColor(float r, float g, float b, float a);

  static void captureScreenshot();

  bool wireframeMode = false;

private:
  Shader m_bodyShader;

  float m_clearColor[4] = {0.1f, 0.1f, 0.1f, 1.0f};

  bool m_initialized = false;
};


#endif // RENDERER_H
