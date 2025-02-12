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

  return true;
}

void ImGuiManager::renderGui(GLFWwindow *window, Renderer &renderer, Camera &camera, PhysicsEngine &physicsEngine) {
  // Start a new ImGui frame
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();

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

  if (ImGui::Button("Start Simulation")) {
    physicsEngine.start();
  }
  ImGui::SameLine();
  if (ImGui::Button("Stop Simulation")) {
    physicsEngine.stop();
  }

  if (ImGui::Button("Step Simulation")) {
    physicsEngine.step();
  }

  ImGui::SameLine();
  if (ImGui::Button("Reset Simulation")) {
    // physicsEngine.reset();
  }

  // Static variables to store user-selected force values
  static double forceX = 0.0;
  static double forceY = 0.0;
  static double forceZ = -9.81;

  static double maxForce = 1000.0;
  static double minForce = -1000.0;

  // Add sliders for each component of the force
  ImGui::Text("Set External Acceleration:");
  ImGui::SliderScalar("X m/s^2", ImGuiDataType_Double, &forceX, &minForce, &maxForce, "%.1f");
  ImGui::SliderScalar("Y m/s^2", ImGuiDataType_Double, &forceY, &minForce, &maxForce, "%.1f");
  ImGui::SliderScalar("Z m/s^2", ImGuiDataType_Double, &forceZ, &minForce, &maxForce, "%.1f");

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

      // Rotation controls (around local x, y, z axes)
      float rotationAngles[3] = {0.0f, 0.0f, 0.0f}; // Angles in radians
      if (ImGui::DragFloat3(
          (label + " Rotation (X, Y, Z)").c_str(),
          rotationAngles,
          0.01f,
          -3.14f,
          3.14f,
          "%.2f rad"
      )) {
        
        // Get the current orientation (w, x, y, z)
        Eigen::Vector4d currentOrientation = body.getOrientation();

        // Create quaternions for each axis rotation
        auto createQuaternion = [](double angle, const Eigen::Vector3d &axis) -> Eigen::Vector4d {
          double halfAngle = angle * 0.5;
          double sinHalfAngle = sin(halfAngle);
          double cosHalfAngle = cos(halfAngle);
          return Eigen::Vector4d(
              cosHalfAngle,
              axis.x() * sinHalfAngle,
              axis.y() * sinHalfAngle,
              axis.z() * sinHalfAngle
          );
        };

        // Quaternions for rotations around x, y, z axes
        Eigen::Vector4d rotX = createQuaternion(rotationAngles[0], Eigen::Vector3d::UnitX());
        Eigen::Vector4d rotY = createQuaternion(rotationAngles[1], Eigen::Vector3d::UnitY());
        Eigen::Vector4d rotZ = createQuaternion(rotationAngles[2], Eigen::Vector3d::UnitZ());

        // Combine the rotations (order matters: Z -> Y -> X)
        auto multiplyQuaternions = [](const Eigen::Vector4d &q1, const Eigen::Vector4d &q2) -> Eigen::Vector4d {
          double w1 = q1(0), x1 = q1(1), y1 = q1(2), z1 = q1(3);
          double w2 = q2(0), x2 = q2(1), y2 = q2(2), z2 = q2(3);
          return Eigen::Vector4d(
              w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, // w
              w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, // x
              w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, // y
              w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2  // z
          );
        };

        // Apply rotations in Z -> Y -> X order
        Eigen::Vector4d newQuaternion = multiplyQuaternions(
            rotZ,
            multiplyQuaternions(
                rotY,
                multiplyQuaternions(
                    rotX,
                    currentOrientation
                )
            )
        );

        // Normalize the quaternion
        double norm = sqrt(newQuaternion(0) * newQuaternion(0) +
                           newQuaternion(1) * newQuaternion(1) +
                           newQuaternion(2) * newQuaternion(2) +
                           newQuaternion(3) * newQuaternion(3));
        newQuaternion /= norm;

        // Update the body's orientation
        body.setOrientation(newQuaternion);
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
