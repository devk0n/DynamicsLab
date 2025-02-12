#include "InputManager.h"
#include "Camera.h"

GLFWwindow *InputManager::s_window = nullptr;
float InputManager::s_scrollOffset = 0.0f;

bool InputManager::initialize(GLFWwindow *window) {
  s_window = window;

  glfwSetScrollCallback(s_window, InputManager::scrollCallback);

  return true;
}

void InputManager::update(float deltaTime, Camera &camera) {
  handleKeyboardInput(deltaTime, camera);
  handleMouseLook(camera);
  handleScrollInput(camera);
  handleUtilityKeybindings();
}

// Keyboard input handling
void InputManager::handleKeyboardInput(float deltaTime, Camera &camera) {
  if (glfwGetKey(s_window, GLFW_KEY_W) == GLFW_PRESS)
    camera.moveForward(deltaTime);
  if (glfwGetKey(s_window, GLFW_KEY_S) == GLFW_PRESS)
    camera.moveBackward(deltaTime);
  if (glfwGetKey(s_window, GLFW_KEY_A) == GLFW_PRESS)
    camera.moveLeft(deltaTime);
  if (glfwGetKey(s_window, GLFW_KEY_D) == GLFW_PRESS)
    camera.moveRight(deltaTime);
  if (glfwGetKey(s_window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
    camera.moveUp(deltaTime);
  if (glfwGetKey(s_window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS)
    camera.moveDown(deltaTime);
}

// Mouse look (only when right mouse button is pressed)
void InputManager::handleMouseLook(Camera &camera) {
  static bool firstMouse = true;
  static float lastX = 0.0f, lastY = 0.0f;

  if (glfwGetMouseButton(s_window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS) {
    // Hide and lock cursor to center of screen
    glfwSetInputMode(s_window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    double xpos, ypos;
    glfwGetCursorPos(s_window, &xpos, &ypos);

    if (firstMouse) {
      lastX = static_cast<float>(xpos);
      lastY = static_cast<float>(ypos);
      firstMouse = false;
    }

    float xOffset = static_cast<float>(xpos) - lastX;
    float yOffset = lastY - static_cast<float>(ypos); // reversed Y

    lastX = static_cast<float>(xpos);
    lastY = static_cast<float>(ypos);

    camera.processMouseMovement(xOffset, yOffset);
  } else {
    // Restore normal cursor when right button is released
    glfwSetInputMode(s_window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
    firstMouse = true;
  }
}

// Scroll wheel => adjust camera speed
void InputManager::handleScrollInput(Camera &camera) {
  if (s_scrollOffset != 0.0f) {
    float currentSpeed = camera.getMovementSpeed();
    float newSpeed = currentSpeed + s_scrollOffset * 0.2f;

    // Clamp speed
    newSpeed = std::max(0.1f, std::min(newSpeed, 30.0f));

    camera.setMovementSpeed(newSpeed);

    // Reset scroll offset after applying
    s_scrollOffset = 0.0f;
  }
}

// Utility keybindings
void InputManager::handleUtilityKeybindings() {
  if (glfwGetKey(s_window, GLFW_KEY_F12) == GLFW_PRESS) {
    Renderer::captureScreenshot();
  }
}

void InputManager::scrollCallback(GLFWwindow *window, double xOffset, double yOffset) {
  s_scrollOffset = static_cast<float>(yOffset);
}

