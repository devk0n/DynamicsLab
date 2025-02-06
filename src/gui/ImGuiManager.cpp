#include "ImGuiManager.h"
#include <iostream>

bool ImGuiManager::initialize(GLFWwindow* window) {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();

    if (!ImGui_ImplGlfw_InitForOpenGL(window, true)) {
        std::cerr << "Error: Failed to initialize ImGui GLFW backend." << std::endl;
        return false;
    }

    constexpr const char* glsl_version = "#version 330";
    if (!ImGui_ImplOpenGL3_Init(glsl_version)) {
        std::cerr << "Error: Failed to initialize ImGui OpenGL3 backend." << std::endl;
        return false;
    }

    return true;
}

void ImGuiManager::beginFrame() {
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
}

void ImGuiManager::performanceWindow() const {
    ImGui::SetNextWindowPos({64, 64}, ImGuiCond_Once);
    ImGui::SetNextWindowSize({256, 128}, ImGuiCond_Once);

    ImGui::Begin("Performance Stats", nullptr, m_windowFlags);

    ImGuiIO& io = ImGui::GetIO();
    ImGui::Text("FPS: %.0f", io.Framerate);
    ImGui::Text("Frame Time: %.3f ms", 1000.0f / io.Framerate);

    ImGui::End();
}

void ImGuiManager::endFrame() {
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void ImGuiManager::shutdown() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}



