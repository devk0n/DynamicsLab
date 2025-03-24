#ifndef WINDOW_MANAGER_H
#define WINDOW_MANAGER_H

#include <string>

#include "OpenGLCore.h"

struct WindowData {
  int width = 0;
  int height = 0;
  std::string title;
  GLFWmonitor *primaryMonitor = nullptr;
};

class WindowManager {
public:
  WindowManager();
  ~WindowManager();

};

#endif // WINDOW_MANAGER_H
