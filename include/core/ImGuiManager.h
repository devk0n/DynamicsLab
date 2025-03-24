#ifndef IMGUI_MANAGER_H
#define IMGUI_MANAGER_H

#include "OpenGLCore.h"

class ImGuiManager {
public:
  bool initialize(GLFWwindow *window);

  void beginFrame() const;
  void endFrame() const;

  void shutdown();

private:
  bool m_initialized = false;
};

#endif // IMGUI_MANAGER_H
