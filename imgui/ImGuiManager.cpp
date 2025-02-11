#include "ImGuiManager.h"
#include "core/Camera.h"


bool ImGuiManager::initialize(GLFWwindow *window) {
  // Setup ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  (void) io;

  // Setup ImGui style
  ImGui::StyleColorsDark();

  // Initialize ImGui for GLFW and OpenGL
  if (!ImGui_ImplGlfw_InitForOpenGL(window, true)) {
    return false;
  }
  if (!ImGui_ImplOpenGL3_Init("#version 460")) {
    return false;
  }

  // ImGuiStyle &style = ImGui::GetStyle();

  return true;
}

void ImGuiManager::renderGui(GLFWwindow *window, Renderer &renderer, Camera &camera, PhysicsEngine &physicsEngine) {
  // Start a new ImGui frame
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();

  // Show a demo window (optional)
  // ImGui::ShowDemoWindow();

  // Show custom UI elements
  showDebugWindow(camera, physicsEngine, window);
  showRendererControls(renderer);
  showCameraControls(camera);
  showPhysicsControls(physicsEngine);
  controlWindow(physicsEngine);

  // Render ImGui
  ImGui::Render();
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void ImGuiManager::showCameraControls(Camera &camera) {
  ImGui::Begin("Camera Controls");
  ImGui::Text("Camera Position");
  ImGui::Text("%.2f, %.2f, %.2f", camera.getPosition().x, camera.getPosition().y, camera.getPosition().z);

  ImGui::Text("Camera Front");
  ImGui::Text("%.2f, %.2f, %.2f", camera.getFront().x, camera.getFront().y, camera.getFront().z);
  // ImGui::Text("Camera Up: (%.2f, %.2f, %.2f)", camera.getUp().x, camera.getUp().y, camera.getUp().z);
  ImGui::Text("Camera Yaw: %.2f degrees", camera.getYaw());
  ImGui::Text("Camera Pitch: %.2f degrees", camera.getPitch());
  ImGui::Text("Camera Movement Speed: %.2f", camera.getMovementSpeed());
  ImGui::Text("Camera Mouse Sensitivity: %.2f", camera.getMouseSensitivity());
  ImGui::End();
}

void ImGuiManager::showRendererControls(Renderer &renderer) {
  ImGui::Begin("Renderer Controls");

  // 1. You need somewhere to store/edit the color.
  //    Often you'll read from the renderer, or store it statically.
  static float clearColor[4] = {0.1f, 0.1f, 0.1f, 1.0f};

  // 2. Provide an ImGui color edit widget
  if (ImGui::ColorEdit3("Background Color", clearColor, ImGuiColorEditFlags_NoInputs)) {
    // This block is called when the user changes the color
    renderer.setClearColor(clearColor[0], clearColor[1], clearColor[2], clearColor[3]);
  }

  static bool wireframeMode = renderer.getWireframeMode();
  if (ImGui::Checkbox("Wireframe Mode", &wireframeMode)) {
    renderer.setWireframeMode(wireframeMode);
  }

  ImGui::End();
}

void ImGuiManager::controlWindow(PhysicsEngine &physicsEngine) {
  ImGui::Begin("Simulation Controls");

  if (ImGui::Button("Initialize Simulation")) {
    physicsEngine.initialize();
  }
  ImGui::SameLine();
  if (ImGui::Button("Stop Simulation")) {
    // m_dynamics->stop();
  }

  if (ImGui::Button("Reset Simulation")) {
    // m_dynamics->reset();
  }

  ImGui::SameLine();
  if (ImGui::Button("Step Simulation")) {
    physicsEngine.step();
  }

  // Static variables to store user-selected force values
  static double forceX = 0.0;
  static double forceY = 0.0;
  static double forceZ = 0.0;

  static double maxForce = 100.0;
  static double minForce = -100.0;

  // Add sliders for each component of the force
  ImGui::Text("Set External Forces:");
  ImGui::SliderScalar("Force X", ImGuiDataType_Double, &forceX, &maxForce, &minForce, "%.1f");
  ImGui::SliderScalar("Force Y", ImGuiDataType_Double, &forceY, &maxForce, &minForce, "%.1f");
  ImGui::SliderScalar("Force Z", ImGuiDataType_Double, &forceZ, &maxForce, &minForce, "%.1f");

  if (ImGui::Button("Reset Force & Torque")) {
    forceX = 0.0;
    forceY = 0.0;
    forceZ = 0.0;
  }

  Eigen::Vector3d externalForces;

  externalForces.setZero();
  externalForces[0] = forceX;
  externalForces[1] = forceY;
  externalForces[2] = forceZ;

  physicsEngine.setExternalForces(externalForces);

  ImGui::End();
}

void ImGuiManager::showPhysicsControls(PhysicsEngine &physicsEngine) {
  ImGui::Begin("Physics Controls");

  auto &rigidBodies = physicsEngine.getRigidBodies();
  for (size_t i = 0; i < rigidBodies.size(); ++i) {
    auto &body = rigidBodies[i];
    std::string label = "RigidBody " + std::to_string(i);

    if (ImGui::CollapsingHeader(label.c_str())) {
      // Position controls
      Eigen::Vector3d position = body.getPosition();
      float pos[3] = {static_cast<float>(position.x()),
                      static_cast<float>(position.y()),
                      static_cast<float>(position.z())};
      if (ImGui::DragFloat3((label + " Position").c_str(), pos, 0.1f)) {
        body.setPosition(Eigen::Vector3d(pos[0], pos[1], pos[2]));
      }

      // Rotation controls - Convert Quaternion to Roll, Pitch, Yaw
      Eigen::Vector4d quatVector = body.getOrientation();
      Eigen::Quaterniond quaternion(quatVector[0], quatVector[1], quatVector[2], quatVector[3]);

      // Extract Euler Angles from Quaternion
      double roll, pitch, yaw;
      Eigen::Matrix3d rotMatrix = quaternion.toRotationMatrix();

      // Extract roll (X), pitch (Y), yaw (Z) correctly from rotation matrix
      pitch = asin(-rotMatrix(2, 0));
      if (std::abs(rotMatrix(2, 0)) < 0.9999) { // Avoid Gimbal Lock
        roll = atan2(rotMatrix(2, 1), rotMatrix(2, 2));
        yaw = atan2(rotMatrix(1, 0), rotMatrix(0, 0));
      } else {
        // Gimbal lock case: We force roll = 0
        roll = 0;
        yaw = atan2(-rotMatrix(0, 1), rotMatrix(1, 1));
      }

      float euler[3] = {static_cast<float>(roll),
                        static_cast<float>(pitch),
                        static_cast<float>(yaw)};

      if (ImGui::DragFloat3((label + " Rotation").c_str(), euler, 0.01f)) {
        // Convert Euler angles back to a Quaternion
        Eigen::Quaterniond newQuaternion =
            Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ()) *  // Yaw
            Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *  // Pitch
            Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX());   // Roll

        Eigen::Vector4d newQuatVector(newQuaternion.w(),
                                      newQuaternion.x(),
                                      newQuaternion.y(),
                                      newQuaternion.z());
        body.setOrientation(newQuatVector);
      }
    }
  }

  ImGui::End();
}


void ImGuiManager::showDebugWindow(Camera &camera, PhysicsEngine &physicsEngine, GLFWwindow *window) {
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

  double currentTime = glfwGetTime();
  frameCount++;

  if (currentTime - lastTime >= 1.0) { // Every second
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


void ImGuiManager::shutdown() {
  // Cleanup ImGui
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
}
