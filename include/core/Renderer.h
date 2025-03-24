#ifndef RENDERER_H
#define RENDERER_H

#include "OpenGLCore.h"

class ShaderManager;
class Camera;

class Renderer {
public:
  Renderer();
  ~Renderer();

  // Initialize global states, load shaders, and create common VAOs/VBOs.
  bool initialize();

  // Frame management: clear the screen at the beginning of a frame.
  static void beginFrame();
  static void clearScreen();
  static void endFrame();

  // Utility drawing functions that environments can use.
  void drawGrid(const Camera &camera) const;

  // Access to the shader manager if needed.
  ShaderManager &getShaderManager() const;

private:
  std::unique_ptr<ShaderManager> m_shaderManager;

  // VAOs and VBOs for common primitives.
  GLuint m_gridVAO, m_gridVBO;

  // Helper functions for initializing the grid and sky resources.
  bool initGrid();
};

#endif // RENDERER_H
