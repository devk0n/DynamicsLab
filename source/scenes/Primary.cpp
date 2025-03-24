#include "Primary.h"

#include "InputManager.h"
#include "WindowManager.h"
#include "ShaderManager.h"
#include "Renderer.h"
#include "Context.h"
#include "PCH.h"
#include "Logger.h"
#include "FrameTimer.h"

bool Primary::load() {

  LOG_INFO("Initializing Primary Scene");
  m_camera.setPosition(glm::vec3(10.0f, 8.0f, 4.0f));
  m_camera.lookAt(glm::vec3(0.0f, 0.0f, 0.0f));
  m_camera.setMovementSpeed(20.0f);

  return true;
}

void Primary::update(double dt) {
  handleCameraMovement(dt);
}

void Primary::render() {
  m_ctx.renderer->drawGrid(m_camera);
  showUI();
}

void Primary::unload() {
  LOG_INFO("Unloading primary scene...");
  m_camera = Camera{};
}

void Primary::showUI() {
  const float currentFps = ImGui::GetIO().Framerate;

  // Initialize displayed FPS on first frame
  if (m_displayedFps == 0.0f) {
    m_displayedFps = currentFps;
  }

  // Update the displayed FPS value once per second
  m_fpsUpdateTimer += ImGui::GetIO().DeltaTime; // Using ImGui's delta time
  if (m_fpsUpdateTimer >= FPS_UPDATE_INTERVAL) {
    m_displayedFps = currentFps;
    m_fpsUpdateTimer = 0.0f;
  }

  const double totalSeconds = m_ctx.frameTimer->getElapsedTime();
  const int hours = static_cast<int>(totalSeconds) / 3600;
  const int minutes = (static_cast<int>(totalSeconds) % 3600) / 60;
  const int seconds = static_cast<int>(totalSeconds) % 60;
  const int milliseconds = static_cast<int>((totalSeconds - std::floor(totalSeconds)) * 1000);

  ImGui::Begin("Window Debug");
  ImGui::Text("FPS: %.0f (%.2f ms)", m_displayedFps, ImGui::GetIO().DeltaTime * 1000);
  ImGui::Text("Elapsed Time: %02d:%02d:%02d.%03d", hours, minutes, seconds, milliseconds);
  ImGui::Text("Delta Time: %.0f µs", m_ctx.frameTimer->getDeltaTime() * 1000000);
  ImGui::End();
}

void Primary::handleCameraMovement(const double dt) {
  // Movement controls
  if (m_ctx.input->isKeyHeld(GLFW_KEY_W))
    m_camera.processKeyboardInput(CameraMovement::FORWARD, dt);
  if (m_ctx.input->isKeyHeld(GLFW_KEY_S))
    m_camera.processKeyboardInput(CameraMovement::BACKWARD, dt);
  if (m_ctx.input->isKeyHeld(GLFW_KEY_A))
    m_camera.processKeyboardInput(CameraMovement::LEFT, dt);
  if (m_ctx.input->isKeyHeld(GLFW_KEY_D))
    m_camera.processKeyboardInput(CameraMovement::RIGHT, dt);
  if (m_ctx.input->isKeyHeld(GLFW_KEY_LEFT_SHIFT))
    m_camera.processKeyboardInput(CameraMovement::UP, dt);
  if (m_ctx.input->isKeyHeld(GLFW_KEY_LEFT_CONTROL))
    m_camera.processKeyboardInput(CameraMovement::DOWN, dt);

  // Right-click look control
  const bool looking = m_ctx.input->isMouseButtonHeld(GLFW_MOUSE_BUTTON_RIGHT);
  glfwSetInputMode(m_ctx.window->getNativeWindow(), GLFW_CURSOR,
                   looking ? GLFW_CURSOR_DISABLED : GLFW_CURSOR_NORMAL);

  if (looking) {
    double xOffset, yOffset;
    m_ctx.input->getMouseDelta(xOffset, yOffset);
    m_camera.processMouseMovement(static_cast<float>(xOffset),
                                  static_cast<float>(yOffset));
  }

  // Handle scroll independently of looking mode
  double xScrollOffset = 0.0, yScrollOffset = 0.0;
  m_ctx.input->getScrollDelta(xScrollOffset, yScrollOffset);
  m_camera.processScroll(static_cast<float>(yScrollOffset));
  // Changed to yScrollOffset
}
