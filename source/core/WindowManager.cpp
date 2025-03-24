#include "WindowManager.h"
#include "PCH.h"
#include "Logger.h"

WindowManager::WindowManager(const std::string_view title)
    : m_window(nullptr, glfwDestroyWindow),
      m_data{0, 0, std::string(title), nullptr} {}

WindowManager::~WindowManager() = default;

bool WindowManager::initialize() {
  if (!glfwInit()) {
    LOG_ERROR("Failed to initialize GLFW");
    return false;
  }

  m_data.primaryMonitor = glfwGetPrimaryMonitor();
  const GLFWvidmode *videoMode = glfwGetVideoMode(m_data.primaryMonitor);
  if (!videoMode) {
    LOG_ERROR("Failed to get video mode");
    return false;
  }

  calculateWindowSize(videoMode);
  setOpenGLHints();

  if (!createWindow())
    return false;
  centerWindow(m_window.get(), videoMode);

  if (!initializeGLAD())
    return false;
  logDebugInfo(videoMode);

  return true;
}

void WindowManager::pollEvents() { glfwPollEvents(); }
void WindowManager::swapBuffers() const { glfwSwapBuffers(m_window.get()); }
bool WindowManager::shouldClose() const { return glfwWindowShouldClose(m_window.get()); }
void WindowManager::close() const { glfwSetWindowShouldClose(m_window.get(), true); }
GLFWwindow *WindowManager::getNativeWindow() const { return m_window.get(); }

void WindowManager::calculateWindowSize(const GLFWvidmode *videoMode) {
  m_data.width = static_cast<int>(videoMode->width * 0.8);
  m_data.height = static_cast<int>(videoMode->height * 0.8);
  m_data.width = std::max(m_data.width, 800);
  m_data.height = std::max(m_data.height, 600);
}

void WindowManager::centerWindow(GLFWwindow *window, const GLFWvidmode *videoMode) const {
  const int xPos = (videoMode->width - m_data.width) / 2;
  const int yPos = (videoMode->height - m_data.height) / 2;
  glfwSetWindowPos(window, xPos, yPos);
}

void WindowManager::logDebugInfo(const GLFWvidmode *videoMode) const {
  LOG_INFO("Window initialized - Size: ", m_data.width, "x", m_data.height);
  LOG_DEBUG("Monitor: ", videoMode->width, "x", videoMode->height, " @ ", videoMode->refreshRate, "Hz");

  if (glGetString(GL_VERSION) == nullptr) {
    LOG_ERROR("ERROR: OpenGL context is not available! glGetString(GL_VERSION) "
              "returned nullptr.");
    return;
  }

  LOG_DEBUG("OpenGL Version: ", reinterpret_cast<const char *>(glGetString(GL_VERSION)));
  LOG_DEBUG("Renderer: ", reinterpret_cast<const char *>(glGetString(GL_RENDERER)));
}

bool WindowManager::createWindow() {
  GLFWwindow *window = glfwCreateWindow(
      m_data.width,
      m_data.height,
      m_data.title.c_str(),
      nullptr,
      nullptr
  );

  if (!window) {
    LOG_ERROR("Failed to create GLFW window");
    glfwTerminate();
    return false;
  }

  m_window.reset(window);
  glfwMakeContextCurrent(m_window.get());
  return true;
}

bool WindowManager::initializeGLAD() {
  if (!gladLoadGLLoader(reinterpret_cast<GLADloadproc>(glfwGetProcAddress))) {
    LOG_ERROR("Failed to initialize GLAD");
    glfwTerminate();
    return false;
  }
  return true;
}

void WindowManager::setOpenGLHints() {
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);
  glfwWindowHint(GLFW_SAMPLES, 4);
  glfwWindowHint(GLFW_DECORATED, GLFW_TRUE);
}
