#include "GuiManager.h"

GuiManager::GuiManager(Window& window) {
    // Initialize ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    ImGui_ImplGlfw_InitForOpenGL(window.getWindow(), true);
    ImGui_ImplOpenGL3_Init("#version 460"); // Adjust for OpenGL version
}

GuiManager::~GuiManager() {
    // Cleanup ImGui
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}

void GuiManager::renderGui() {
    beginFrame();
    performanceWindow();

    endFrame();
}

void GuiManager::performanceWindow() const {

    ImGui::SetNextWindowPos(ImVec2(64, 64), ImGuiCond_Once);
    ImGui::SetNextWindowSize(ImVec2(256, 128), ImGuiCond_Once);

    ImGui::Begin("Performance Stats", nullptr, m_windowFlags);

    // Retrieve current FPS
    ImGuiIO& io = ImGui::GetIO();
    double fps = io.Framerate;

    // Calculate approximate frame time in milliseconds
    double frameTimeMs = 1000.0 / fps;

    // Display the values
    ImGui::Text("FPS: %.0f", fps);
    ImGui::Text("Frame Time: %.3f ms", frameTimeMs);

    ImGui::End();

}

void GuiManager::beginFrame() {
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
}

void GuiManager::endFrame() {
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}