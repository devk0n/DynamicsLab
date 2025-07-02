#include "Primary.h"
#include "DynamicsBuilder.h"
#include "FrameTimer.h"
#include "InputManager.h"
#include "PCH.h"
#include "WindowManager.h"

using namespace Proton;

void Primary::singlePendulum() {
  const DynamicsBuilder builder(m_system);

  Body* b1 = builder.createBody()
    .position(0, 0, 5)
    .fixed(true)
    .build();

  Body* b2 = builder.createBody()
    .position(1,6,5)
    .build();

  builder.createGravity()
    .setGravity(0, 0, -9.81)
    .addBody(b1)
    .addBody(b2)
    .build();

  builder.createDistanceConstraint()
    .withAutoDistance(true)
    .withBodyA(b1)
    .withBodyB(b2)
    .build();
}

void Primary::doublePendulum() {
  const DynamicsBuilder builder(m_system);

  Body* b1 = builder.createBody()
    .position(0, 0, 5)
    .size(5, 0.5, 0.5)
    .fixed(true)
    .build();

  Body* b2 = builder.createBody()
    .position(5,0,5)
    .build();

  Body* b3 = builder.createBody()
    .position(5, 0, 10)
    .build();

  builder.createGravity()
    .setGravity(0, 0, -9.81)
    .addBody(b1)
    .addBody(b2)
    .addBody(b3)
    .build();

  builder.createSphericalJoint()
    .between(b1, b2)
    .withLocalPointA(2.5, 0, 0)
    .withLocalPointB(-2.5, 0, 0)
    .build();

  builder.createSphericalJoint()
    .between(b2, b3)
    .withLocalPointA(0, 0, 2.5)
    .withLocalPointB(0, 0, -2.5)
    .build();
}



bool Primary::load() {

  // doublePendulum();

  // stressCarpet();

  stressCarpetNoDiagonals();
    
  LOG_INFO("Initializing Primary Scene");
  m_camera.setPosition({5.0f, 6.2f, 6.2f});
  m_camera.lookAt({0.0f, 0.0f, 5.0f});
  m_camera.setMovementSpeed(5.0f);

  //FIXME: Full path too files are temporary because of move from Windows to Linux development!
  if (!m_ctx.renderer->getShaderManager()
      .loadShader("bodyShader",
                  "/home/devkon/Projects/DynamicsLab/assets/shaders/body.vert",
                  "/home/devkon/Projects/DynamicsLab/assets/shaders/body.frag")) {
    LOG_ERROR("Failed to load body shader");
    return false;
  }

  if (!m_ctx.renderer->getShaderManager()
        .loadShader("lineShader",
                    "/home/devkon/Projects/DynamicsLab/assets/shaders/line.vert",
                    "/home/devkon/Projects/DynamicsLab/assets/shaders/line.frag")) {
    LOG_ERROR("Failed to load line shader");
    return false;
  }

  return true;
}

// This is not a "working" suspension for a car, but rather a function
// demo for the application

void Primary::vehicleDemo() {
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

void Primary::stressCarpet() {
    const DynamicsBuilder builder(m_system);

    constexpr int N = 5;         // Grid width/height
    constexpr double spacing = 1.0; // Distance between nodes
    constexpr double startX = 0.0;
    constexpr double startY = 0.0;
    constexpr double startZ = 6.0;

    // Store grid of pointers to bodies
    Body* bodies[N][N];

    // Create bodies in a grid
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            bool isPinned = (i == 0 && j == 0) || (i == 0 && j == N-1); // Pin two top corners
            bodies[i][j] = builder.createBody()
                .position(startX + i * spacing, startY + j * spacing, startZ)
                .size(0.6, 0.6, 0.6)
                .mass(1.0)
                .fixed(isPinned)
                .build();
        }
    }

    // Add gravity to all bodies
    auto grav = builder.createGravity().setGravity(0, 0, -9.81);
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j)
            grav.addBody(bodies[i][j]);
    grav.build();

    // Add distance constraints to immediate neighbors
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            // Right neighbor
            if (i + 1 < N)
                builder.createDistanceConstraint()
                    .withBodyA(bodies[i][j])
                    .withBodyB(bodies[i+1][j])
                    .withAutoDistance(true)
                    .build();
            // Down neighbor
            if (j + 1 < N)
                builder.createDistanceConstraint()
                    .withBodyA(bodies[i][j])
                    .withBodyB(bodies[i][j+1])
                    .withAutoDistance(true)
                    .build();
            // Optionally, diagonals for max stress:
            if (i + 1 < N && j + 1 < N)
                builder.createDistanceConstraint()
                    .withBodyA(bodies[i][j])
                    .withBodyB(bodies[i+1][j+1])
                    .withAutoDistance(true)
                    .build();
            if (i + 1 < N && j - 1 >= 0)
                builder.createDistanceConstraint()
                    .withBodyA(bodies[i][j])
                    .withBodyB(bodies[i+1][j-1])
                    .withAutoDistance(true)
                    .build();
        }
    }
}


void Primary::stressCarpetNoDiagonals() {
    const DynamicsBuilder builder(m_system);

    constexpr int N = 5;         // Grid width/height
    constexpr double spacing = 1.0; // Distance between nodes
    constexpr double startX = 0.0;
    constexpr double startY = 0.0;
    constexpr double startZ = 6.0;

    // Store grid of pointers to bodies
    Body* bodies[N][N];

    // Create bodies in a grid
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            bool isPinned = (i == 0 && j == 0) || (i == 0 && j == N-1); // Pin two top corners
            bodies[i][j] = builder.createBody()
                .position(startX + i * spacing, startY + j * spacing, startZ)
                .size(0.6, 0.6, 0.6)
                .mass(1.0)
                .fixed(isPinned)
                .build();
        }
    }

    // Add gravity to all bodies
    auto grav = builder.createGravity().setGravity(0, 0, -9.81);
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j)
            grav.addBody(bodies[i][j]);
    grav.build();

    // Only right and down neighbor constraints (no diagonals)
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            // Right neighbor
            if (i + 1 < N)
                builder.createDistanceConstraint()
                    .withBodyA(bodies[i][j])
                    .withBodyB(bodies[i+1][j])
                    .withAutoDistance(true)
                    .build();
            // Down neighbor
            if (j + 1 < N)
                builder.createDistanceConstraint()
                    .withBodyA(bodies[i][j])
                    .withBodyB(bodies[i][j+1])
                    .withAutoDistance(true)
                    .build();
        }
    }
}

