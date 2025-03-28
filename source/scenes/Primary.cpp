#include "Primary.h"

#include "RevoluteJoint.h"

#include "BallJoint.h"
#include "GravityForce.h"
#include "InputManager.h"
#include "WindowManager.h"
#include "Dynamics.h"
#include "Renderer.h"
#include "Context.h"
#include "PCH.h"
#include "Logger.h"
#include "FrameTimer.h"
#include "Torque.h"

using namespace Proton;

void Primary::setupDynamics() {

  UniqueID body0 = m_system.addBody(
    6,
    {1, 1, 1},
    {0, 0, 1},
    {1, 0, 0, 0}
  );

  UniqueID body1 = m_system.addBody(
    6,
    {1, 0.5, 0.5},
    {0, 0, 1},
    {1, 0, 0, 0}
  );

  Body* b0 = m_system.getBody(body0);
  Body* b1 = m_system.getBody(body1);

  b1->setSize({1, 0.2, 0.2});
  b1->setAngularVelocity({25,0,0});

  auto gravity = std::make_shared<GravityForce>(Vector3d(0, 0, -9.81));
  gravity->addBody(b1);
  m_system.addForceGenerator(gravity);

  m_system.addConstraint(std::make_shared<BallJoint>(
    b0, Vector3d( 0.5, 0, 0),
    b1, Vector3d(-0.5, 0, 0)
  ));

  b0->setFixed(true);
  b0->setSize({0.1, 0.1, 0.1});
}

bool Primary::load() {

  setupDynamics();

  LOG_INFO("Initializing Primary Scene");
  m_camera.setPosition(glm::vec3(5.0f, 3.2f, 3.2f));
  m_camera.lookAt(glm::vec3(0.0f, 0.0f, 2.0f));
  m_camera.setMovementSpeed(5.0f);

  if (!m_ctx.renderer->getShaderManager()
      .loadShader("cubeShader",
                  "assets/shaders/cube.vert",
                  "assets/shaders/cube.frag")) {
    LOG_ERROR("Failed to load cube shader");
    return false;
  }

  if (!m_ctx.renderer->getShaderManager()
        .loadShader("lineShader",
                    "assets/shaders/line.vert",
                    "assets/shaders/line.frag")) {
    LOG_ERROR("Failed to load line shader");
    return false;
  }

  return true;
}

void Primary::update(double dt) {
  handleCameraMovement(dt);

  if (m_ctx.input->isKeyPressed(GLFW_KEY_SPACE)) { toggle(m_run); }

  if (m_ctx.input->isKeyPressed(GLFW_KEY_L)) {
    Body* b1 = m_system.getBody(0);
    m_camera.lookAt(b1->getPositionVec3());
  }

  if (m_run) {
    m_system.step(dt);
  }
}

void Primary::render() {
  showUI();

  m_systemVisualizer.render(m_system, m_camera.getPosition(), m_camera.getViewMatrix(), m_camera.getProjectionMatrix());
  m_ctx.renderer->drawGrid(m_camera);
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
  ImGui::Text("Status: %s", m_run ? "Running" : "Stopped");
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
