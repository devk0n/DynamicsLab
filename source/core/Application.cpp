#include "Application.h"

#include <Primary.h>

#include "ImGuiManager.h"
#include "SceneManager.h"
#include "Context.h"
#include "Renderer.h"
#include "WindowManager.h"
#include "InputManager.h"
#include "Logger.h"
#include "FrameTimer.h"

Application::Application()
  : m_ctx(std::make_unique<Context>()),
    m_windowManager(std::make_unique<WindowManager>("DynamicsLab")),
    m_inputManager(std::make_unique<InputManager>()),
    m_renderer(std::make_unique<Renderer>()),
    m_imguiManager(std::make_unique<ImGuiManager>()),
    m_sceneManager(std::make_unique<SceneManager>()),
    m_frameTimer(std::make_unique<FrameTimer>()) {}

Application::~Application() {
  m_imguiManager->shutdown();
  LOG_INFO("Application destroyed");
  m_windowManager.reset();
  glfwTerminate();
  LOG_DEBUG("GLFW terminated");
}

bool Application::initialize() {
  if (!initializeWindow()) return false;
  if (!initializeInput()) return false;
  if (!initializeRenderer()) return false;
  if (!initializeImGui()) return false;

  setupContext();

  // Load initial scene
  m_sceneManager->pushScene(std::make_unique<Primary>(*m_ctx));

  m_initialized = true;
  return true;
}

void Application::setupContext() {
  m_ctx->window = m_windowManager.get();
  m_ctx->input = m_inputManager.get();
  m_ctx->renderer = m_renderer.get();
  m_ctx->imgui = m_imguiManager.get();
  m_ctx->scene = m_sceneManager.get();
  m_ctx->frameTimer = m_frameTimer.get();
}

bool Application::initializeWindow() const {
  if (!m_windowManager->initialize()) {
    LOG_ERROR("Failed to initialize window manager");
    return false;
  }
  return true;
}

bool Application::initializeInput() const {
  if (!m_inputManager->initialize(m_windowManager->getNativeWindow())) {
    LOG_ERROR("Failed to initialize input manager");
    return false;
  }
  return true;
}

bool Application::initializeRenderer() const {
  if (!m_renderer->initialize()) {
    LOG_ERROR("Failed to initialize renderer");
    return false;
  }
  return true;
}

bool Application::initializeImGui() const {
  if (!m_imguiManager->initialize(m_windowManager->getNativeWindow())) {
    LOG_ERROR("Failed to initialize ImGui manager");
    return false;
  }
  return true;
}

void Application::run() {
  if (!m_initialized) {
    LOG_ERROR("Application not initialized!");
    return;
  }

  LOG_INFO("Application started");
  m_frameTimer->update();

  while (!m_windowManager->shouldClose()) {
    m_frameTimer->update();

    WindowManager::pollEvents();
    Renderer::beginFrame();
    m_sceneManager->update(m_frameTimer->getDeltaTime());
    m_inputManager->update();
    Renderer::endFrame();

    m_imguiManager->beginFrame();
    m_sceneManager->render();
    m_imguiManager->endFrame();

    m_windowManager->swapBuffers();
  }

  LOG_INFO("Application closed");
}

