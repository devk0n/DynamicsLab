#ifndef WINDOW_MANAGER_H
#define WINDOW_MANAGER_H

#include <memory>
#include <string>

#include "OpenGLCore.h"

// Data structure for window properties
struct WindowData {
  int          width{0};
  int          height{0};
  std::string  title;
  GLFWmonitor* primaryMonitor{nullptr};

};

class WindowManager {
public:
  explicit WindowManager(std::string_view title);
  ~WindowManager();

  // Core Interface
  [[nodiscard]] bool initialize();
  [[nodiscard]] bool shouldClose() const;

  void swapBuffers() const;
  void close() const;

  static void pollEvents();

  [[nodiscard]] GLFWwindow* getNativeWindow() const;

private:
  std::unique_ptr<GLFWwindow, void (*)(GLFWwindow*)> m_window;
  WindowData m_data;

  // Initialization Helpers
  void calculateWindowSize(const GLFWvidmode* videoMode);
  void centerWindow(GLFWwindow* window, const GLFWvidmode* videoMode) const;
  void logDebugInfo(const GLFWvidmode* videoMode) const;

  bool createWindow();

  void setResizeCallback(std::function<void(int, int)> callback);

  static bool initializeGLAD();
  static void setOpenGLHints();
};

#endif // WINDOW_MANAGER_H
