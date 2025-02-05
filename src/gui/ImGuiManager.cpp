//
// Created by devkon on 05/02/2025.
//
#include <iostream>

#include "ImGuiManager.h"

bool ImGuiManager::initialize(GLFWwindow *window) {
    // Check that ImGui version is correct and create a new ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    (void)io; // Prevent unused variable warning if you don't use 'io' immediately

    // Set ImGui style (e.g., Dark mode)
    ImGui::StyleColorsDark();

    // Initialize platform and renderer bindings for GLFW and OpenGL3
    if (!ImGui_ImplGlfw_InitForOpenGL(window, true)) {
        std::cerr << "Failed to initialize ImGui GLFW backend." << std::endl;
        return false;
    }

    // You may need to adjust the GLSL version string depending on your setup.
    const char* glsl_version = "#version 330";
    if (!ImGui_ImplOpenGL3_Init(glsl_version)) {
        std::cerr << "Failed to initialize ImGui OpenGL3 backend." << std::endl;
        return false;
    }

    return true;
}

void ImGuiManager::performanceWindow() const {

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

void ImGuiManager::renderGui() {
    beginFrame();

    performanceWindow();

    endFrame();
}

void ImGuiManager::beginFrame() {
    // Start a new frame for both backends
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
}

void ImGuiManager::endFrame() {
    // Finalize the ImGui frame and render the draw data
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void ImGuiManager::shutdown() {
    // Clean up the ImGui backend resources and destroy the ImGui context
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}



