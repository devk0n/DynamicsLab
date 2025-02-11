#include "ImGuiManager.h"
#include "core/Camera.h"


bool ImGuiManager::initialize(GLFWwindow* window) {
    // Setup ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    (void)io;

    // Setup ImGui style
    ImGui::StyleColorsDark();

    // Initialize ImGui for GLFW and OpenGL
    if (!ImGui_ImplGlfw_InitForOpenGL(window, true)) {
        return false;
    }
    if (!ImGui_ImplOpenGL3_Init("#version 460")) {
        return false;
    }

    ImGuiStyle& style = ImGui::GetStyle();

    return true;
}

void ImGuiManager::renderGui(GLFWwindow* window, Renderer& renderer, Camera& camera, PhysicsEngine& physicsEngine) {
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

    // Render ImGui
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void ImGuiManager::showCameraControls(Camera& camera) {
    ImGui::Begin("Camera Controls");
    ImGui::Text("Camera Position: (%.2f, %.2f, %.2f)", camera.getPosition().x, camera.getPosition().y, camera.getPosition().z);
    ImGui::Text("Camera Front: (%.2f, %.2f, %.2f)", camera.getFront().x, camera.getFront().y, camera.getFront().z);
    ImGui::End();
}

void ImGuiManager::showRendererControls(Renderer& renderer) {
    ImGui::Begin("Renderer Controls");

    // 1. You need somewhere to store/edit the color.
    //    Often you'll read from the renderer, or store it statically.
    static float clearColor[4] = { 0.1f, 0.1f, 0.1f, 1.0f };

    // 2. Provide an ImGui color edit widget
    if (ImGui::ColorEdit3("Background Color", clearColor, ImGuiColorEditFlags_NoInputs)) {
        // This block is called when the user changes the color
        renderer.setClearColor(clearColor[0], clearColor[1], clearColor[2], clearColor[3]);
    }

    ImGui::End();
}

void ImGuiManager::showPhysicsControls(PhysicsEngine& physicsEngine) {
    ImGui::Begin("Physics Controls");

    auto& rigidBodies = physicsEngine.getRigidBodies();
    for (size_t i = 0; i < rigidBodies.size(); ++i) {
        auto& body = rigidBodies[i];
        std::string label = "RigidBody " + std::to_string(i);

        if (ImGui::CollapsingHeader(label.c_str())) {
            // Position controls
            Eigen::Vector3d position = body.getPosition();
            float pos[3] = { static_cast<float>(position.x()), static_cast<float>(position.y()), static_cast<float>(position.z()) };
            if (ImGui::DragFloat3((label + " Position").c_str(), pos, 0.1f)) {
                body.setPosition(Eigen::Vector3d(pos[0], pos[1], pos[2]));
            }

            // Rotation controls (Convert stored Quaternion to Euler angles for UI)
            Eigen::Vector4d quatVector = body.getOrientation();
            Eigen::Quaterniond quaternion(quatVector[0], quatVector[1], quatVector[2], quatVector[3]);
            Eigen::Vector3d eulerRotation = quaternion.toRotationMatrix().eulerAngles(0, 1, 2);

            float euler[3] = { static_cast<float>(eulerRotation.x()), static_cast<float>(eulerRotation.y()), static_cast<float>(eulerRotation.z()) };
            if (ImGui::DragFloat3((label + " Rotation").c_str(), euler, 0.01f)) {
                Eigen::Quaterniond newQuaternion = Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()) *
                                                   Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
                                                   Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ());
                Eigen::Vector4d newQuatVector(newQuaternion.w(), newQuaternion.x(), newQuaternion.y(), newQuaternion.z());
                body.setOrientation(newQuatVector);
            }
        }
    }

    ImGui::End();
}



void ImGuiManager::showDebugWindow(Camera& camera, PhysicsEngine& physicsEngine, GLFWwindow* window) {
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
