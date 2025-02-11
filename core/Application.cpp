#include "Application.h"
#include "physics/PhysicsEngine.h"

std::vector<Vertex> cubeVertices = {
  // Front face
  {{-0.5f, -0.5f,  0.5f}, {0.0f, 0.0f, 1.0f}}, // Bottom-left
  {{0.5f, -0.5f,  0.5f}, {0.0f, 0.0f, 1.0f}},  // Bottom-right
  {{0.5f,  0.5f,  0.5f}, {0.0f, 0.0f, 1.0f}},  // Top-right
  {{-0.5f,  0.5f,  0.5f}, {0.0f, 0.0f, 1.0f}}, // Top-left

  // Back face
  {{-0.5f, -0.5f, -0.5f}, {0.0f, 0.0f, -1.0f}}, // Bottom-left
  {{0.5f, -0.5f, -0.5f}, {0.0f, 0.0f, -1.0f}},  // Bottom-right
  {{0.5f,  0.5f, -0.5f}, {0.0f, 0.0f, -1.0f}},  // Top-right
  {{-0.5f,  0.5f, -0.5f}, {0.0f, 0.0f, -1.0f}}  // Top-left
};

std::vector<GLuint> cubeIndices = {
  // Front face
  0, 1, 2,
  2, 3, 0,

  // Back face
  4, 5, 6,
  6, 7, 4,

  // Left face
  4, 0, 3,
  3, 7, 4,

  // Right face
  1, 5, 6,
  6, 2, 1,

  // Top face
  3, 2, 6,
  6, 7, 3,

  // Bottom face
  4, 5, 1,
  1, 0, 4
};

Application::Application() : m_lastFrameTime(0.0f) {}

bool Application::initialize() {
  if (!m_windowManager.initialize()) return false;
  if (!m_imGuiManager.initialize(m_windowManager.getWindow())) return false;
  if (!InputManager::initialize(m_windowManager.getWindow())) return false;

  if (!m_renderer.initialize("C:/Users/devkon/CLionProjects/DynamicsLab/assets/shaders/cube.vert.glsl", "C:/Users/devkon/CLionProjects/DynamicsLab/assets/shaders/cube.frag.glsl")) {
    std::cerr << "Failed to initialize renderer" << std::endl;
    return false;
  }

  // Create a cube
  RigidBody cube(Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Vector4d(1.0, 0.0, 0.0, 0.0), cubeVertices, cubeIndices);
  m_physicsEngine.addRigidBody(cube);

  RigidBody cube1(Eigen::Vector3d(10.0, 0.0, 0.0), Eigen::Vector4d(1.0, 0.0, 0.0, 0.0), cubeVertices, cubeIndices);
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
    float deltaTime = static_cast<float>(glfwGetTime()) - m_lastFrameTime;
    m_lastFrameTime = static_cast<float>(glfwGetTime());

    m_windowManager.pollEvents();
    InputManager::update(deltaTime, m_camera);

    m_renderer.beginFrame();
    m_renderer.render(
    m_physicsEngine.getRigidBodies(),
    m_camera.getViewMatrix(),
    m_camera.getProjectionMatrix(m_windowManager.getAspectRatio())
    );

    // Render ImGui controls
    m_imGuiManager.renderGui(m_windowManager.getWindow(), m_renderer, m_camera, m_physicsEngine);

    m_renderer.endFrame();
    m_windowManager.swapBuffers();
  }
}

