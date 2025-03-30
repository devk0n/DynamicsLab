#include "Application.h"
#include "Primary.h"
#include "ImGuiManager.h"
#include "SceneManager.h"
#include "Context.h"
#include "Renderer.h"
#include "WindowManager.h"
#include "InputManager.h"
#include "Logger.h"
#include "FrameTimer.h"
#include "ScopedTimer.h"

struct Application::Impl {
  std::unique_ptr<Context> m_ctx;
  std::unique_ptr<WindowManager> m_windowManager;
  std::unique_ptr<InputManager> m_inputManager;
  std::unique_ptr<Renderer> m_renderer;
  std::unique_ptr<ImGuiManager> m_imguiManager;
  std::unique_ptr<SceneManager> m_sceneManager;
  std::unique_ptr<FrameTimer> m_frameTimer;
  bool m_initialized = false;

  Impl();
  ~Impl();

  bool initialize();
  void run() const;
  void setupContext() const;
  [[nodiscard]] bool initializeWindow() const;
  [[nodiscard]] bool initializeInput() const;
  [[nodiscard]] bool initializeRenderer() const;
  [[nodiscard]] bool initializeImGui() const;
};

// Application methods
Application::Application() : m_impl(std::make_unique<Impl>()) {}
Application::~Application() = default;

bool Application::initialize() const { return m_impl->initialize(); }
void Application::run() const { m_impl->run(); }

// Impl methods
Application::Impl::Impl()
  : m_ctx(std::make_unique<Context>()),
    m_windowManager(std::make_unique<WindowManager>("DynamicsLab")),
    m_inputManager(std::make_unique<InputManager>()),
    m_renderer(std::make_unique<Renderer>()),
    m_imguiManager(std::make_unique<ImGuiManager>()),
    m_sceneManager(std::make_unique<SceneManager>()),
    m_frameTimer(std::make_unique<FrameTimer>()) {}

Application::Impl::~Impl() {
  m_imguiManager->shutdown();
  LOG_INFO("Application destroyed");
  glfwTerminate();
  LOG_DEBUG("GLFW terminated");
}

bool Application::Impl::initialize() {
  if (m_initialized) return true;

  if (!initializeWindow()) return false;
  if (!initializeInput()) return false;
  if (!initializeRenderer()) return false;
  if (!initializeImGui()) return false;

  setupContext();
  m_sceneManager->pushScene(std::make_unique<Primary>(*m_ctx));
  m_initialized = true;
  return true;
}

void Application::Impl::setupContext() const {
  m_ctx->window = m_windowManager.get();
  m_ctx->input = m_inputManager.get();
  m_ctx->renderer = m_renderer.get();
  m_ctx->imgui = m_imguiManager.get();
  m_ctx->scene = m_sceneManager.get();
  m_ctx->frameTimer = m_frameTimer.get();
}

bool Application::Impl::initializeWindow() const {
  if (!m_windowManager->initialize()) {
    LOG_ERROR("Failed to initialize window manager");
    return false;
  }
  return true;
}

bool Application::Impl::initializeInput() const {
  if (!m_inputManager->initialize(m_windowManager->getNativeWindow())) {
    LOG_ERROR("Failed to initialize input manager");
    return false;
  }
  return true;
}

bool Application::Impl::initializeRenderer() const {
  if (!m_renderer->initialize()) {
    LOG_ERROR("Failed to initialize renderer");
    return false;
  }
  return true;
}

bool Application::Impl::initializeImGui() const {
  if (!m_imguiManager->initialize(m_windowManager->getNativeWindow())) {
    LOG_ERROR("Failed to initialize ImGui manager");
    return false;
  }
  return true;
}

void Application::Impl::run() const {
  if (!m_initialized) {
    LOG_ERROR("Application not initialized!");
    return;
  }

  LOG_INFO("Application started");
  m_frameTimer->update();

  while (!m_windowManager->shouldClose()) {
    m_frameTimer->update();
    m_frameTimer->clearTimings();
    {
      ScopedTimer t("PollEvents", *m_frameTimer);
      WindowManager::pollEvents();
    }
    {
      ScopedTimer t("BeginFrame", *m_frameTimer);
      Renderer::beginFrame();
    }
    {
      ScopedTimer t("UpdateScene", *m_frameTimer);
      m_sceneManager->update(m_frameTimer->getDeltaTime());
    }
    {
      ScopedTimer t("UpdateInput", *m_frameTimer);
      m_inputManager->update();
    }
    {
      ScopedTimer t("EndFrame", *m_frameTimer);
      Renderer::endFrame();
    }
    {
      ScopedTimer t("ImGuiBegin", *m_frameTimer);
      m_imguiManager->beginFrame();
    }
    {
      ScopedTimer t("RenderScene", *m_frameTimer);
      m_sceneManager->render();
    }
    {
      ScopedTimer t("ImGuiEnd", *m_frameTimer);
      m_imguiManager->endFrame();
    }
    {
      ScopedTimer t("SwapBuffers", *m_frameTimer);
      m_windowManager->swapBuffers();
    }
  }
  LOG_INFO("Application closed");
}