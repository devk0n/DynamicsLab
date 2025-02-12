#ifndef INPUTMANAGER_H
#define INPUTMANAGER_H

#include "Camera.h"
#include "graphics/Renderer.h"
#include <GLFW/glfw3.h>

class InputManager {
public:
  static bool initialize(GLFWwindow *window);

  static void update(float deltaTime, Camera &camera);

private:
  static GLFWwindow *s_window;

  static float s_scrollOffset;

  static void scrollCallback(GLFWwindow *window, double xOffset, double yOffset);

  static void handleKeyboardInput(float time, Camera &camera);

  static void handleMouseLook(Camera &camera);

  static void handleScrollInput(Camera &camera);

  static void handleUtilityKeybindings();
};


#endif // INPUTMANAGER_H
