#ifndef APPLICATION_H
#define APPLICATION_H

#include "WindowManager.h"
#include "graphics/Renderer.h"

#include "Camera.h"
#include "InputManager.h"

#include "physics/PhysicsEngine.h"
#include "imgui/ImGuiManager.h"

class Application {
public:
  Application();

  bool initialize();

  void run();

private:
  WindowManager m_windowManager;
  Renderer m_renderer;
  PhysicsEngine m_physicsEngine;
  ImGuiManager m_imGuiManager;
  Camera m_camera;

  float m_lastFrameTime;

  void update(float deltaTime);

};

#endif // APPLICATION_H