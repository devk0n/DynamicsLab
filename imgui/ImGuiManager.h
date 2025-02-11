#ifndef IMGUIMANAGER_H
#define IMGUIMANAGER_H

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "glm/glm.hpp"

#include "physics/PhysicsEngine.h"
#include "graphics/Renderer.h"
#include "core/Camera.h"
#include <GLFW/glfw3.h>

class ImGuiManager {
public:
  bool initialize(GLFWwindow *window);

  void renderGui(GLFWwindow *window, Renderer &renderer, Camera &camera, PhysicsEngine &physicsEngine);

private:
  // ImGui context and state
  static void showRendererControls(Renderer &renderer);

  static void showCameraControls(Camera &camera);

  static void showPhysicsControls(PhysicsEngine &physicsEngine);

  static void showDebugWindow(Camera &camera, PhysicsEngine &physicsEngine, GLFWwindow *window);

  void shutdown();


};


#endif // IMGUIMANAGER_H
