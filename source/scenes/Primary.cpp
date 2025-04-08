#include "Primary.h"
#include "DynamicsBuilder.h"
#include "FrameTimer.h"
#include "InputManager.h"
#include "PCH.h"
#include "WindowManager.h"

using namespace Proton;

void Primary::problemA() {
  const DynamicsBuilder builder(m_system);

  Body* chassis = builder.createBody()
    .size(3.0, 0.5, 0.5)
    .mass(300)
    .position(0, 0, 2)
    .fixed(true)
    .build();

  Body* LF = builder.createBody()
    .mass(10)
    .size(0.25, 0.25, 0.5)
    .position(1.2, 1, 2)
    .build();

  Body* LR = builder.createBody()
    .mass(10)
    .size(0.25, 0.25, 0.5)
    .position(-1.2, 1, 2)
    .build();

  Body* RF = builder.createBody()
    .mass(10)
    .size(0.25, 0.25, 0.5)
    .position(1.2, -1, 2)
    .build();

  Body* RR = builder.createBody()
    .mass(10)
    .size(0.25, 0.25, 0.5)
    .position(-1.2, -1, 2)
    .build();

  builder.createGravity(0.0, 0.0, 9.81)
    .addBody(LF)
    .addBody(LR)
    .addBody(RF)
    .addBody(RR)
    .build();

  builder.createSpring()
    .withBodyA(chassis)
    .withBodyB(LF)
    .withLocalPointA(1.2, 0.25, 0.25)
    .withLocalPointB(0, 0, -0.25)
    .withAutoDistance(true)
    .withStiffness(700)
    .withDamping(50)
    .build();

  builder.createSpring()
    .withBodyA(chassis)
    .withBodyB(RF)
    .withLocalPointA(1.2, -0.25, 0.25)
    .withLocalPointB(0, 0, -0.25)
    .withAutoDistance(true)
    .withStiffness(700)
    .withDamping(50)
    .build();

  builder.createSpring()
    .withBodyA(chassis)
    .withBodyB(RR)
    .withLocalPointA(-1.2, -0.25, 0.25)
    .withLocalPointB(0, 0, -0.25)
    .withAutoDistance(true)
    .withStiffness(700)
    .withDamping(50)
    .build();

  builder.createSpring()
    .withBodyA(chassis)
    .withBodyB(LR)
    .withLocalPointA(-1.2, 0.25, 0.25)
    .withLocalPointB(0, 0, -0.25)
    .withAutoDistance(true)
    .withStiffness(700)
    .withDamping(50)
    .build();

  // Left Front
  builder.createDistanceConstraint()
    .withBodyA(chassis)
    .withBodyB(LF)
    .withLocalPointA(1.1, 0.25, 0)
    .withLocalPointB(-0.2, 0, 0)
    .withAutoDistance(true)
    .build();

  builder.createDistanceConstraint()
    .withBodyA(chassis)
    .withBodyB(LF)
    .withLocalPointA(1.5, 0.25, 0.25)
    .withLocalPointB(0, 0, 0.25)
    .withAutoDistance(true)
    .build();

  builder.createDistanceConstraint()
    .withBodyA(chassis)
    .withBodyB(LF)
    .withLocalPointA(0.9, 0.25, 0.25)
    .withLocalPointB(0, 0, 0.25)
    .withAutoDistance(true)
    .build();

  builder.createDistanceConstraint()
    .withBodyA(chassis)
    .withBodyB(LF)
    .withLocalPointA(1.5, 0.25, -0.25)
    .withLocalPointB(0, 0, -0.25)
    .withAutoDistance(true)
    .build();

  builder.createDistanceConstraint()
    .withBodyA(chassis)
    .withBodyB(LF)
    .withLocalPointA(0.9, 0.25, -0.25)
    .withLocalPointB(0, 0, -0.25)
    .withAutoDistance(true)
    .build();

  // Right Front
  builder.createDistanceConstraint()
    .withBodyA(chassis)
    .withBodyB(RF)
    .withLocalPointA(1.1, -0.25, 0)
    .withLocalPointB(-0.2, 0, 0)
    .withAutoDistance(true)
    .build();

  builder.createDistanceConstraint()
    .withBodyA(chassis)
    .withBodyB(RF)
    .withLocalPointA(1.5, -0.25, 0.25)
    .withLocalPointB(0, 0, 0.25)
    .withAutoDistance(true)
    .build();

  builder.createDistanceConstraint()
    .withBodyA(chassis)
    .withBodyB(RF)
    .withLocalPointA(0.9, -0.25, 0.25)
    .withLocalPointB(0, 0, 0.25)
    .withAutoDistance(true)
    .build();

  builder.createDistanceConstraint()
    .withBodyA(chassis)
    .withBodyB(RF)
    .withLocalPointA(1.5, -0.25, -0.25)
    .withLocalPointB(0, 0, -0.25)
    .withAutoDistance(true)
    .build();

  builder.createDistanceConstraint()
    .withBodyA(chassis)
    .withBodyB(RF)
    .withLocalPointA(0.9, -0.25, -0.25)
    .withLocalPointB(0, 0, -0.25)
    .withAutoDistance(true)
    .build();

  // Right Rear
  builder.createDistanceConstraint()
    .withBodyA(chassis)
    .withBodyB(RR)
    .withLocalPointA(-1.5, -0.25, -0.25)
    .withLocalPointB(-0.10, 0, -0.25)
    .withAutoDistance(true)
    .build();

  builder.createDistanceConstraint()
    .withBodyA(chassis)
    .withBodyB(RR)
    .withLocalPointA(-1.5, -0.25, 0.25)
    .withLocalPointB(0, 0, 0.25)
    .withAutoDistance(true)
    .build();

  builder.createDistanceConstraint()
    .withBodyA(chassis)
    .withBodyB(RR)
    .withLocalPointA(-0.9, -0.25, 0.25)
    .withLocalPointB(0, 0, 0.25)
    .withAutoDistance(true)
    .build();

  builder.createDistanceConstraint()
    .withBodyA(chassis)
    .withBodyB(RR)
    .withLocalPointA(-1.5, -0.25, -0.25)
    .withLocalPointB(0, 0, -0.25)
    .withAutoDistance(true)
    .build();

  builder.createDistanceConstraint()
    .withBodyA(chassis)
    .withBodyB(RR)
    .withLocalPointA(-0.9, -0.25, -0.25)
    .withLocalPointB(0, 0, -0.25)
    .withAutoDistance(true)
    .build();

  // Left Rear
  builder.createDistanceConstraint()
    .withBodyA(chassis)
    .withBodyB(LR)
    .withLocalPointA(-1.5, 0.25, -0.25)
    .withLocalPointB(-0.1, 0, -0.25)
    .withAutoDistance(true)
    .build();

  builder.createDistanceConstraint()
    .withBodyA(chassis)
    .withBodyB(LR)
    .withLocalPointA(-1.5, 0.25, 0.25)
    .withLocalPointB(0, 0, 0.25)
    .withAutoDistance(true)
    .build();

  builder.createDistanceConstraint()
    .withBodyA(chassis)
    .withBodyB(LR)
    .withLocalPointA(-0.9, 0.25, 0.25)
    .withLocalPointB(0, 0, 0.25)
    .withAutoDistance(true)
    .build();

  builder.createDistanceConstraint()
    .withBodyA(chassis)
    .withBodyB(LR)
    .withLocalPointA(-1.5, 0.25, -0.25)
    .withLocalPointB(0, 0, -0.25)
    .withAutoDistance(true)
    .build();

  builder.createDistanceConstraint()
    .withBodyA(chassis)
    .withBodyB(LR)
    .withLocalPointA(-0.9, 0.25, -0.25)
    .withLocalPointB(0, 0, -0.25)
    .withAutoDistance(true)
    .build();


}

void Primary::problemB() {
  const DynamicsBuilder builder(m_system);

  Body* anchor = builder.createBody()
    .size(0.01, 0.01, 0.01)
    .fixed(true)
    .build();

  Body* arm1 = builder.createBody()
    .mass(8)
    .size(0.2, 0.04, 0.04)
    .position(0.1, 0.0, 0.0)
    .build();

  Body* arm2 = builder.createBody()
    .mass(12)
    .size(0.6, 0.03, 0.03)
    .orientation(0, -38.51, 58.43)
    .position(0.322903645, 0.200, 0.186801215)
    .build();

  Body* arm3 = builder.createBody()
    .mass(12)
    .size(0.5, 0.03, 0.03)
    .orientation(0, 26.92, 0)
    .position(0.222903645, 0.400, 0.486801215)
    .build();

  builder.createSphericalJoint()
    .withBodyA(anchor)
    .withBodyB(arm1)
    .withLocalPointA(0, 0, 0)
    .withLocalPointB(-0.1, 0, 0)
    .build();

  builder.createSphericalJoint()
    .withBodyA(arm1)
    .withBodyB(arm2)
    .withLocalPointA(0.1, 0, 0)
    .withLocalPointB(-0.3, 0, 0)
    .build();

  builder.createSphericalJoint()
    .withBodyA(arm2)
    .withBodyB(arm3)
    .withLocalPointA(0.3, 0.0, 0.0)
    .withLocalPointB(0.25, 0.0, 0.0)
    .build();

  builder.createSphericalJoint()
    .withBodyA(anchor)
    .withBodyB(arm3)
    .withLocalPointA(0, 0.4, 0.6)
    .withLocalPointB(-0.25, 0, 0)
    .build();

  builder.createGravity(0, 0, -9.81)
    .addBody(arm1)
    .addBody(arm2)
    .addBody(arm3)
    .build();
}

void Primary::doublePendulum() {
  const DynamicsBuilder builder(m_system);

  Body* anchor = builder.createBody()
    .position(0, 0, 5.0)
    .size(0.1, 0.1, 0.1)
    .fixed(true)
    .build();

  Body* arm1 = builder.createBody()
    .mass(10)
    .size(3.0, 0.2, 0.2)
    .position(1.5, 0.0, 5.0)
    .build();

  Body* arm2 = builder.createBody()
    .geometryType(GeometryType::Cylinder)
    .mass(5)
    .size(2.0, 2.0, 2.0)
    .inertia(0.075, 1.704, 1.704)
    .position(3.0, .0, 6.0)
    .orientation(0, -90, 0)
    .color(0.0f, 0.5f, 1.0f)
    .angularVelocity(0.1, 0.0, 0.0)
    .build();

  builder.createSphericalJoint()
    .withBodyA(anchor)
    .withBodyB(arm1)
    .withLocalPointA(0, 0, 0)
    .withLocalPointB(-1.5, 0, 0)
    .build();

  builder.createSphericalJoint()
    .withBodyA(arm1)
    .withBodyB(arm2)
    .withLocalPointA(1.5, 0, 0)
    .withLocalPointB(-1, 0, 0)
    .build();

  builder.createGravity(0, 0, -9.81)
    .addBody(arm1)
    .addBody(arm2)
    .build();
}

void Primary::problemC() {
  const DynamicsBuilder builder(m_system);

  Body* anchor = builder.createBody()
    .position(-1.0, 0, 0)
    .size(0.1, 0.1, 0.1)
    .fixed(true)
    .build();

  Body* arm1 = builder.createBody()
    .position(1, 0, 0)
    .size(2, 0.4, 0.4)
    .mass(8)
    .build();

  builder.createUniversalJoint()
    .between(anchor, arm1)
    .withLocalPointA(1, 0, 0)
    .withLocalPointB(-1, 0, 0)
    .withAxisA(0, 1, 0)
    .withAxisB(0, 1, 0)
    .build();

  builder.createGravity(0, 0, -9.81)
    .addBody(arm1)
    .build();
}

void Primary::vehicle() {
  const DynamicsBuilder system(m_system);

  Body* anchor = system.createBody()
    .position(0, 0, 0)
    .size(0.1, 0.1, 0.1)
    .fixed(true)
    .build();

  Body* arm1 = system.createBody()
    .position(1.5 * cosd(45), 1.5 * cosd(45), 0)
    .size(3, 0.3, 0.3)
    .orientation(0, 0, 45)
    .mass(100)
    .build();

  Body* arm2 = system.createBody()
    .position(3 * cosd(45), 1.5 * cosd(45), 0)
    .size(3 * cosd(45), 0.3, 0.3)
    .orientation(0, 0, -90)
    .mass(100)
    .build();

  system.createUniversalJoint()
    .between(anchor, arm1)
    .withLocalPointA(0, 0, 0)
    .withLocalPointB(-1.5, 0, 0)
    .withAxisA(0, 0, 1)
    .withAxisB(0, 1, 0)
    .build();

  system.createRevoluteJoint()
    .between(arm1, arm2)
    .withLocalPointA(1.5, 0, 0)
    .withLocalPointB(-1.5 * cosd(45), 0, 0)
    .withAxis(0, 1, 0)
    .build();

  system.createGravity(0, 0, -9.81)
    .addBody(arm1)
    .addBody(arm2)
    .build();
}


bool Primary::load() {

  // problemA();
  // problemB();
  // problemC();
  // doublePendulum();

  vehicle();

  LOG_INFO("Initializing Primary Scene");
  m_camera.setPosition({5.0f, 3.2f, 3.2f});
  m_camera.lookAt({0.0f, 0.0f, 0.0f});
  m_camera.setMovementSpeed(5.0f);

  if (!m_ctx.renderer->getShaderManager()
      .loadShader("bodyShader",
                  "../assets/shaders/body.vert",
                  "../assets/shaders/body.frag")) {
    LOG_ERROR("Failed to load body shader");
    return false;
  }

  if (!m_ctx.renderer->getShaderManager()
        .loadShader("lineShader",
                    "../assets/shaders/line.vert",
                    "../assets/shaders/line.frag")) {
    LOG_ERROR("Failed to load line shader");
    return false;
  }

  return true;
}

void Primary::update(const double dt) {
  handleCameraMovement(dt);

  if (m_ctx.input->isKeyPressed(GLFW_KEY_SPACE)) { toggle(m_run); }

  if (m_ctx.input->isKeyHeld(GLFW_KEY_L)) {
    const Body* b1 = m_system.getBody(0);
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
  renderTimings(m_ctx.frameTimer->getTimings());
  ImGui::End();

}

void Primary::renderTimings(const std::vector<std::pair<std::string, double>>& timings) {
  std::unordered_map<std::string, double> merged;

  for (const auto& [label, time] : timings) {
    merged[label] += time; // or use max(time, merged[label]) for peak
  }

  for (const auto& [label, total] : merged) {
    ImGui::Text("%-20s : %7.3f ms", label.c_str(), total);
  }
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
}
