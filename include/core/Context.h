#ifndef CONTEXT_H
#define CONTEXT_H

// Forward declarations
class WindowManager;
class InputManager;
class Renderer;
class SceneManager;
class ImGuiManager;
class FrameTimer;

struct Context {
  WindowManager* window = nullptr;
  InputManager* input = nullptr;
  Renderer* renderer = nullptr;
  SceneManager* scene = nullptr;
  ImGuiManager* imgui = nullptr;
  FrameTimer* frameTimer = nullptr;
};

#endif // CONTEXT_H
