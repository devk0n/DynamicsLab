#ifndef INPUT_MANAGER_H
#define INPUT_MANAGER_H

#include <unordered_set>

#include "OpenGLCore.h"

class InputManager {
public:
  InputManager();

  bool initialize(GLFWwindow *window);
  void update();

  // Keyboard
  [[nodiscard]] bool isKeyPressed(int key) const;
  [[nodiscard]] bool isKeyHeld(int key) const;
  [[nodiscard]] bool isKeyReleased(int key) const;

  // Mouse
  [[nodiscard]] bool isMouseButtonPressed(int button) const;
  [[nodiscard]] bool isMouseButtonHeld(int button) const;
  [[nodiscard]] bool isMouseButtonReleased(int button) const;

  void getMousePosition(double &xPos, double &yPos) const;
  void getMouseDelta(double &dx, double &dy) const;
  void getScrollDelta(double &xOffset, double &yOffset) const;

private:
  static void keyCallback(
      GLFWwindow *window,
      int key,
      int scancode,
      int action,
      int mods
  );

  static void mouseButtonCallback(
      GLFWwindow *window,
      int button,
      int action,
      int mods
  );

  static void scrollCallback(
      GLFWwindow *window,
      double xOffset,
      double yOffset
  );

  GLFWwindow *m_window;

  // Keyboard state
  std::unordered_set<int> m_heldKeys;
  std::unordered_set<int> m_pressedKeys;
  std::unordered_set<int> m_releasedKeys;

  // Mouse state
  std::unordered_set<int> m_heldMouseButtons;
  std::unordered_set<int> m_pressedMouseButtons;
  std::unordered_set<int> m_releasedMouseButtons;

  // Mouse position
  double m_currentMouseX = 0.0;
  double m_currentMouseY = 0.0;
  double m_previousMouseX = 0.0;
  double m_previousMouseY = 0.0;

  // Scroll state
  double m_scrollX = 0.0;
  double m_scrollY = 0.0;
  double m_previousScrollX = 0.0;
  double m_previousScrollY = 0.0;
};

#endif // INPUT_MANAGER_H
