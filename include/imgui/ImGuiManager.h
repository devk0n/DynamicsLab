#ifndef IMGUIMANAGER_H
#define IMGUIMANAGER_H

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "glm/glm.hpp"

#include "PhysicsEngine.h"
#include "Renderer.h"
#include "Camera.h"
#include <GLFW/glfw3.h>

class ImGuiManager {
public:
  static bool initialize(GLFWwindow *window);

  void renderGui(GLFWwindow *window, Renderer &renderer, const Camera &camera, PhysicsEngine &physicsEngine);

private:
  // ImGui context and state
  static void showRendererControls(Renderer &renderer);

  static void showCameraControls(const Camera &camera);

  static void showBodyControls(PhysicsEngine &physicsEngine);

  static void showDebugWindow(const Camera &camera, PhysicsEngine &physicsEngine, GLFWwindow *window);

  static void shutdown();

  void showPhysicsControl(PhysicsEngine &physicsEngine);
};


#endif // IMGUIMANAGER_H
