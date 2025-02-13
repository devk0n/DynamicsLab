#include "Application.h"
#include "physics/PhysicsEngine.h"
#include "graphics/MeshData.h"

Application::Application() : m_lastFrameTime(0.0f) {
}

bool Application::initialize() {
  if (!m_windowManager.initialize()) return false;
  if (!ImGuiManager::initialize(m_windowManager.getWindow())) return false;
  if (!InputManager::initialize(m_windowManager.getWindow())) return false;

  if (!m_renderer.initialize()) {
    std::cerr << "Failed to initialize renderer" << std::endl;
    return false;
  }

  // Create a cube
  const RigidBody cube0(
    Eigen::Vector3d(0.0, 0.0, 0.0),
    Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
    Eigen::Matrix3d::Identity() * 10.0,
    Eigen::Matrix3d::Identity() * 60.0,
    MeshData::cubeVertices,
    MeshData::cubeIndices,
    glm::vec3(0.6118, 0.2510, 0.4039)
  );

  m_physicsEngine.addRigidBody(cube0);
  m_physicsEngine.initialize();

  return true;
}

void Application::update(const float deltaTime) {
  InputManager::update(deltaTime, m_camera);

  if (m_physicsEngine.isInitialized() && m_physicsEngine.isRunning()) {
    m_physicsEngine.step();
  }
}

void Application::run() {
  while (!m_windowManager.shouldClose()) {
    const auto currentTime = static_cast<float>(glfwGetTime());
    const float deltaTime = currentTime - m_lastFrameTime;
    m_lastFrameTime = currentTime;

    m_windowManager.pollEvents();
    update(deltaTime);

    m_renderer.beginFrame();
    m_renderer.render(
      m_physicsEngine.getRigidBodies(),
      m_camera.getViewMatrix(),
      m_camera.getProjectionMatrix(m_windowManager.getAspectRatio())
    );

    m_imGuiManager.renderGui(
      m_windowManager.getWindow(),
      m_renderer,
      m_camera,
      m_physicsEngine
    );

    m_renderer.endFrame();
    m_windowManager.swapBuffers();
  }
}
