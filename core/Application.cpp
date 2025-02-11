#include <filesystem>
#include "Application.h"
#include "physics/PhysicsEngine.h"
#include "graphics/MeshData.h"

Application::Application() : m_lastFrameTime(0.0f) {}

bool Application::initialize() {
  if (!m_windowManager.initialize()) return false;
  if (!m_imGuiManager.initialize(m_windowManager.getWindow())) return false;
  if (!InputManager::initialize(m_windowManager.getWindow())) return false;

  std::cout << "Current path: " << std::filesystem::current_path().string() << std::endl;

  if (!m_renderer.initialize("assets/shaders/cube.vert.glsl",
                             "assets/shaders/cube.frag.glsl")) {
    std::cerr << "Failed to initialize renderer" << std::endl;
    return false;
  }
  
  // Create a cube
  RigidBody cube(
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
      MeshData::cubeVertices,
      MeshData::cubeIndices
  );

  m_physicsEngine.addRigidBody(cube);

  RigidBody cube1(
      Eigen::Vector3d(10.0, 0.0, 0.0),
      Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
      MeshData::cubeVertices,
      MeshData::cubeIndices
  );
  m_physicsEngine.addRigidBody(cube1);

  return true;
}

void Application::update(float deltaTime) {
  InputManager::update(deltaTime, m_camera);
  // m_renderer.setViewMatrix(m_camera.getViewMatrix());
  // m_physicsManager.step(deltaTime);
}

void Application::run() {
  while (!m_windowManager.shouldClose()) {
    auto currentTime = static_cast<float>(glfwGetTime());
    float deltaTime = currentTime - m_lastFrameTime;
    m_lastFrameTime = currentTime;

    m_windowManager.pollEvents();
    update(deltaTime);

    m_renderer.beginFrame();
    m_renderer.render(
        m_physicsEngine.getRigidBodies(),
        m_camera.getViewMatrix(),
        m_camera.getProjectionMatrix(m_windowManager.getAspectRatio())
    );

    m_imGuiManager.renderGui(m_windowManager.getWindow(), m_renderer, m_camera, m_physicsEngine);

    m_renderer.endFrame();
    m_windowManager.swapBuffers();
  }
}

