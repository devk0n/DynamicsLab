#include "imgui_layer.h"

#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

ImGuiLayer::ImGuiLayer(GLFWwindow* window) : m_Window(window) {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(m_Window, true);
    ImGui_ImplOpenGL3_Init("#version 330 core");
}

ImGuiLayer::~ImGuiLayer() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}

void ImGuiLayer::renderUI() {
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    // Call individual UI sections here
    showMainMenu();
    showSimulationControls();
    showRenderingOptions();
    showDebugWindow();

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

// Individual UI components

void ImGuiLayer::showMainMenu() {
    ImGui::Begin("Main Menu");
    if (ImGui::Button("Start Simulation")) {
        // Simulation start logic here
    }
    ImGui::SameLine();
    if (ImGui::Button("Stop Simulation")) {
        // Simulation stop logic here
    }
    ImGui::End();
}

void ImGuiLayer::showSimulationControls() {
    ImGui::Begin("Simulation Controls");

    static float simulationSpeed = 1.0f;
    ImGui::SliderFloat("Speed", &simulationSpeed, 0.1f, 10.0f);

    static bool enablePhysics = true;
    ImGui::Checkbox("Enable Physics", &enablePhysics);

    ImGui::End();
}

void ImGuiLayer::showRenderingOptions() {
    ImGui::Begin("Rendering Options");

    static bool showWireframe = false;
    ImGui::Checkbox("Show Wireframe", &showWireframe);
    if (showWireframe) {
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    } else {
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }

    static float bgColor[3] = {0.1f, 0.1f, 0.1f};
    ImGui::ColorEdit3("Background Color", bgColor);
    glClearColor(bgColor[0], bgColor[1], bgColor[2], 1.0f);

    ImGui::End();
}

void ImGuiLayer::showDebugWindow() {
    ImGui::Begin("Debug Info");

    static int frameCount = 0;
    frameCount++;
    ImGui::Text("Frame Count: %d", frameCount);
    ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);

    ImGui::End();
}

