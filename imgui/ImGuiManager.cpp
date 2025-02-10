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
    // Create a window for physics controls
    ImGui::Begin("Physics Controls");

    // Add UI elements for physics controls
    if (ImGui::Button("Reset Simulation")) {
        // Reset physics simulation
        // physicsEngine.reset();
    }

    // Add more controls as needed
    static float gravity = -9.81f;
    if (ImGui::SliderFloat("Gravity", &gravity, -20.0f, 20.0f)) {
        // physicsEngine.setGravity(glm::vec3(0.0f, 0.0f, gravity)); // +Z is up
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
