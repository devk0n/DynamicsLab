#include "InputManager.h"

InputManager::InputManager() : m_window(nullptr) {}

bool InputManager::initialize(GLFWwindow *window) {
  m_window = window;

  // Set GLFW callbacks
  glfwSetKeyCallback(window, keyCallback);
  glfwSetMouseButtonCallback(window, mouseButtonCallback);
  glfwSetScrollCallback(window, scrollCallback);
  glfwSetWindowUserPointer(window, this);

  // Get initial mouse position
  glfwGetCursorPos(window, &m_currentMouseX, &m_currentMouseY);
  m_previousMouseX = m_currentMouseX;
  m_previousMouseY = m_currentMouseY;

  return true;
}

void InputManager::update() {
  // Update mouse position
  m_previousMouseX = m_currentMouseX;
  m_previousMouseY = m_currentMouseY;
  glfwGetCursorPos(m_window, &m_currentMouseX, &m_currentMouseY);

  // Update scroll values
  m_previousScrollX = m_scrollX;
  m_previousScrollY = m_scrollY;

  // Clear single-frame states
  m_pressedKeys.clear();
  m_releasedKeys.clear();
  m_pressedMouseButtons.clear();
  m_releasedMouseButtons.clear();
}

// Keyboard state checks
bool InputManager::isKeyPressed(int key) const {
  return m_pressedKeys.contains(key);
}
bool InputManager::isKeyHeld(int key) const {
  return m_heldKeys.contains(key);
}
bool InputManager::isKeyReleased(int key) const {
  return m_releasedKeys.contains(key);
}

// Mouse state checks
bool InputManager::isMouseButtonPressed(int button) const {
  return m_pressedMouseButtons.contains(button);
}
bool InputManager::isMouseButtonHeld(int button) const {
  return m_heldMouseButtons.contains(button);
}
bool InputManager::isMouseButtonReleased(int button) const {
  return m_releasedMouseButtons.contains(button);
}

void InputManager::getMousePosition(double &xPos, double &yPos) const {
  xPos = m_currentMouseX;
  yPos = m_currentMouseY;
}

void InputManager::getMouseDelta(double &dx, double &dy) const {
  dx = m_currentMouseX - m_previousMouseX;
  dy = m_currentMouseY - m_previousMouseY;
}

void InputManager::getScrollDelta(double &xOffset, double &yOffset) const {
  xOffset = m_scrollX - m_previousScrollX;
  yOffset = m_scrollY - m_previousScrollY;
}

// Keyboard handling
void InputManager::keyCallback(GLFWwindow *window, int key, int scancode,
                               int action, int mods) {
  auto *input = static_cast<InputManager *>(glfwGetWindowUserPointer(window));

  if (action == GLFW_PRESS) {
    input->m_heldKeys.insert(key);
    input->m_pressedKeys.insert(key);
  } else if (action == GLFW_RELEASE) {
    input->m_heldKeys.erase(key);
    input->m_releasedKeys.insert(key);
  }
}

// Mouse button handling
void InputManager::mouseButtonCallback(GLFWwindow *window, int button,
                                       int action, int mods) {
  auto *input = static_cast<InputManager *>(glfwGetWindowUserPointer(window));

  if (action == GLFW_PRESS) {
    input->m_heldMouseButtons.insert(button);
    input->m_pressedMouseButtons.insert(button);
  } else if (action == GLFW_RELEASE) {
    input->m_heldMouseButtons.erase(button);
    input->m_releasedMouseButtons.insert(button);
  }
}

// Scroll handling
void InputManager::scrollCallback(GLFWwindow *window, double xOffset,
                                  double yOffset) {
  auto *input = static_cast<InputManager *>(glfwGetWindowUserPointer(window));
  input->m_scrollX += xOffset;
  input->m_scrollY += yOffset;
}
