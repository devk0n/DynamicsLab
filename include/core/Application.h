#ifndef APPLICATION_H
#define APPLICATION_H

struct Context;
class WindowManager;
class InputManager;
class Renderer;
class SceneManager;
class ImGuiManager;
class FrameTimer;

class Application {
public:
  Application();
  ~Application();

  bool initialize();
  void run();

  void setupContext();

  [[nodiscard]] bool initializeWindow() const;
  [[nodiscard]] bool initializeInput() const;
  [[nodiscard]] bool initializeRenderer() const;

  bool initializeImGui() const;

private:
  std::unique_ptr<Context> m_ctx;

  std::unique_ptr<WindowManager> m_windowManager;
  std::unique_ptr<InputManager> m_inputManager;
  std::unique_ptr<Renderer> m_renderer;
  std::unique_ptr<ImGuiManager> m_imguiManager;
  std::unique_ptr<SceneManager> m_sceneManager;
  std::unique_ptr<FrameTimer> m_frameTimer;

  bool m_initialized = false;
};

#endif // APPLICATION_H
