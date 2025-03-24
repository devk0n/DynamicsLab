#include "ImGuiManager.h"

#include "PCH.h"

bool ImGuiManager::initialize(GLFWwindow *window) {
  // Create ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();

  // Set up ImGui style (optional)
  ImGui::StyleColorsDark();

  // Initialize the ImGui GLFW and OpenGL backends
  if (!ImGui_ImplGlfw_InitForOpenGL(window, true)) {
    return false;
  }
  // Use e.g. "#version 460" or what’s appropriate for your GL version
  ImGui_ImplOpenGL3_Init("#version 460");

  m_initialized = true;
  return m_initialized;
}

void ImGuiManager::beginFrame() const {
  if (!m_initialized)
    return;

  // Start a new frame
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();

  ImGuizmo::BeginFrame();
}

void ImGuiManager::endFrame() const {
  if (!m_initialized)
    return;

  // Render ImGui’s draw data
  ImGui::Render();
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void ImGuiManager::shutdown() {
  if (!m_initialized)
    return;

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
  m_initialized = false;
}
