#include "ImGuiManager.h"

void ImGuiManager::showDebugWindow(const Camera &camera, PhysicsEngine &physicsEngine, GLFWwindow *window) {
  ImGui::Begin("Debug");

  // OpenGL Info
  ImGui::Text("OpenGL Version: %s", glGetString(GL_VERSION));
  ImGui::Text("GLSL Version: %s", glGetString(GL_SHADING_LANGUAGE_VERSION));
  ImGui::Text("Vendor: %s", glGetString(GL_VENDOR));
  ImGui::Text("Renderer: %s", glGetString(GL_RENDERER));

  // FPS Counter
  static double lastTime = 0.0;
  static int frameCount = 0;
  static float fps = 0.0f;
  static float frameTime = 0.0f;

  const double currentTime = glfwGetTime();
  frameCount++;

  if (currentTime - lastTime >= 1.0) {
    // Every second
    fps = frameCount / (currentTime - lastTime);
    frameTime = 1000.0f / fps; // ms per frame
    frameCount = 0;
    lastTime = currentTime;
  }

  ImGui::Separator();
  ImGui::Text("Performance");
  ImGui::Text("FPS: %.1f", fps);
  ImGui::Text("Frame Time: %.2f ms", frameTime);

  // Camera Info
  ImGui::Separator();
  ImGui::Text("Camera");
  ImGui::Text("Position: (%.2f, %.2f, %.2f)", camera.getPosition().x, camera.getPosition().y, camera.getPosition().z);
  // ImGui::Text("Yaw: %.2f, Pitch: %.2f", camera.getYaw(), camera.getPitch());

  // Window Info
  int width, height;
  glfwGetWindowSize(window, &width, &height);
  ImGui::Separator();
  ImGui::Text("Window Size: %d x %d", width, height);
  ImGui::Text("Aspect Ratio: %.2f", static_cast<float>(width) / height);

  // Shader Info
  GLint currentShader = 0;
  glGetIntegerv(GL_CURRENT_PROGRAM, &currentShader);
  ImGui::Text("Active Shader ID: %d", currentShader);

  // Physics Debug
  ImGui::Separator();
  ImGui::Text("Physics Debug");
  // ImGui::Text("Gravity: (%.2f, %.2f, %.2f)", physicsEngine.getGravity().x, physicsEngine.getGravity().y, physicsEngine.getGravity().z);
  // ImGui::Text("Rigid Bodies: %zu", physicsEngine.getRigidBodyCount());
  // ImGui::Text("Physics Step Time: %.3f ms", physicsEngine.getLastStepTime() * 1000.0f);

  // ImGui Metrics
  if (ImGui::CollapsingHeader("ImGui Metrics")) {
    ImGui::ShowMetricsWindow();
  }

  ImGui::End();
}
