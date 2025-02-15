#ifndef APPLICATION_H
#define APPLICATION_H

#include "WindowManager.h"
#include "Renderer.h"

#include "Camera.h"
#include "InputManager.h"

#include "PhysicsEngine.h"
#include "ImGuiManager.h"

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
