#include "Application.h"
#include "physics/PhysicsEngine.h"

// Define cube vertices with correct normals
std::vector<Vertex> cubeVertices = {
    // Front face
    {{-0.5f, -0.5f, 0.5f},  {0.0f,  0.0f,  1.0f}},
    {{0.5f,  -0.5f, 0.5f},  {0.0f,  0.0f,  1.0f}},
    {{0.5f,  0.5f,  0.5f},  {0.0f,  0.0f,  1.0f}},
    {{-0.5f, 0.5f,  0.5f},  {0.0f,  0.0f,  1.0f}},

    // Back face
    {{-0.5f, -0.5f, -0.5f}, {0.0f,  0.0f,  -1.0f}},
    {{0.5f,  -0.5f, -0.5f}, {0.0f,  0.0f,  -1.0f}},
    {{0.5f,  0.5f,  -0.5f}, {0.0f,  0.0f,  -1.0f}},
    {{-0.5f, 0.5f,  -0.5f}, {0.0f,  0.0f,  -1.0f}},

    // Left face
    {{-0.5f, -0.5f, -0.5f}, {-1.0f, 0.0f,  0.0f}},
    {{-0.5f, -0.5f, 0.5f},  {-1.0f, 0.0f,  0.0f}},
    {{-0.5f, 0.5f,  0.5f},  {-1.0f, 0.0f,  0.0f}},
    {{-0.5f, 0.5f,  -0.5f}, {-1.0f, 0.0f,  0.0f}},

    // Right face
    {{0.5f,  -0.5f, -0.5f}, {1.0f,  0.0f,  0.0f}},
    {{0.5f,  -0.5f, 0.5f},  {1.0f,  0.0f,  0.0f}},
    {{0.5f,  0.5f,  0.5f},  {1.0f,  0.0f,  0.0f}},
    {{0.5f,  0.5f,  -0.5f}, {1.0f,  0.0f,  0.0f}},

    // Top face
    {{-0.5f, 0.5f,  -0.5f}, {0.0f,  1.0f,  0.0f}},
    {{-0.5f, 0.5f,  0.5f},  {0.0f,  1.0f,  0.0f}},
    {{0.5f,  0.5f,  0.5f},  {0.0f,  1.0f,  0.0f}},
    {{0.5f,  0.5f,  -0.5f}, {0.0f,  1.0f,  0.0f}},

    // Bottom face
    {{-0.5f, -0.5f, -0.5f}, {0.0f,  -1.0f, 0.0f}},
    {{-0.5f, -0.5f, 0.5f},  {0.0f,  -1.0f, 0.0f}},
    {{0.5f,  -0.5f, 0.5f},  {0.0f,  -1.0f, 0.0f}},
    {{0.5f,  -0.5f, -0.5f}, {0.0f,  -1.0f, 0.0f}}
};

// Indices for drawing the cube using element buffer
std::vector<unsigned int> cubeIndices = {
    0, 1, 2, 2, 3, 0,  // Front
    4, 5, 6, 6, 7, 4,  // Back
    8, 9, 10, 10, 11, 8,  // Left
    12, 13, 14, 14, 15, 12,  // Right
    16, 17, 18, 18, 19, 16,  // Top
    20, 21, 22, 22, 23, 20   // Bottom
};

Application::Application() : m_lastFrameTime(0.0f) {}

bool Application::initialize() {
  if (!m_windowManager.initialize()) return false;
  if (!m_imGuiManager.initialize(m_windowManager.getWindow())) return false;
  if (!InputManager::initialize(m_windowManager.getWindow())) return false;

  if (!m_renderer.initialize("C:/Users/devkon/CLionProjects/DynamicsLab/assets/shaders/cube.vert.glsl",
                             "C:/Users/devkon/CLionProjects/DynamicsLab/assets/shaders/cube.frag.glsl")) {
    std::cerr << "Failed to initialize renderer" << std::endl;
    return false;
  }

  // Create a cube
  RigidBody cube(
      Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
      cubeVertices,
      cubeIndices
  );

  m_physicsEngine.addRigidBody(cube);

  RigidBody cube1(
      Eigen::Vector3d(10.0, 0.0, 0.0),
      Eigen::Vector4d(1.0, 0.0, 0.0, 0.0),
      cubeVertices,
      cubeIndices
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

