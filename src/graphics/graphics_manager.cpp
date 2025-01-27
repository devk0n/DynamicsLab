#include <stdexcept>
#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"
#include <thread>

#include "graphics_manager.h"

GraphicsManager::GraphicsManager()
    : WINDOW_WIDTH(1280),
      WINDOW_HEIGHT(960),
      WINDOW_TITLE("DynamicsLab"),
      window(nullptr, glfwDestroyWindow) {
    initializeGLFW();
    createWindow();
    initializeImGui();
}

GraphicsManager::~GraphicsManager() {
    cleanUp();
}

void GraphicsManager::initializeGLFW() {
    if (!glfwInit()) {
        throw std::runtime_error("Failed to initialize GLFW.");
    }
}

void GraphicsManager::createWindow() {
    glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);
    GLFWwindow* rawWindow = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, WINDOW_TITLE, nullptr, nullptr);
    if (!rawWindow) {
        glfwTerminate();
        throw std::runtime_error("Failed to create GLFW window.");
    }
    window.reset(rawWindow);
}

void GraphicsManager::run() {
    mainLoop();
}

void GraphicsManager::mainLoop() {
    constexpr double targetFrameTime = 1.0 / 60.0; // Target 60 FPS
    while (!glfwWindowShouldClose(window.get())) {
        auto startTime = std::chrono::high_resolution_clock::now();

        glfwPollEvents(); // Process events

        glClear(GL_COLOR_BUFFER_BIT); // Clear buffers
        renderGUI(); // Render GUI
        glfwSwapBuffers(window.get()); // Swap buffers

        auto endTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> frameDuration = endTime - startTime;

        double waitTime = targetFrameTime - frameDuration.count();
        if (waitTime > 0.0) {
            std::this_thread::sleep_for(std::chrono::duration<double>(waitTime));
        }
    }
}

void GraphicsManager::renderGUI() {
    // Start a new ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    showSimulationWindow();
    // showControlWindow();
    showDebugWindow();

    // Render ImGui frame
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

// Function to show the simulation monitoring window
void GraphicsManager::showSimulationWindow() {
    ImGui::Begin("Simulation Monitoring");
    ImGui::Text("Time: %.2f", 0.0);
    ImGui::Text("Position X: %.3f", 1);
    ImGui::Text("Position Y: %.3f", 1);
    ImGui::Text("Position Z: %.3f", 1);
    ImGui::End();
}

// Function to show the control panel
void GraphicsManager::showControlWindow() {
    ImGui::Begin("Control Panel");
    // static double inputVal = 0.00025f;
    // ImGui::InputDouble("Time Step", &inputVal, 0.00025, 0.01, "%.5f", 2);
    // ImGui::Checkbox("Pause", &taskActive);
    ImGui::End();
}

// Function to show a debugging window
void GraphicsManager::showDebugWindow() {
    ImGui::Begin("Debug Info");
    ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);
    ImGui::Text("Window Size: %d x %d", WINDOW_WIDTH, WINDOW_HEIGHT);

    // Retrieve information about another window
    ImVec2 windowPos = ImGui::GetWindowPos();
    ImVec2 windowSize = ImGui::GetWindowSize();

    ImGui::Text("Window Position: (%.1f, %.1f)", windowPos.x, windowPos.y);
    ImGui::Text("Window Size: (%.1f x %.1f)", windowSize.x, windowSize.y);

    ImGui::Text("Time: %.4f", time);

    ImGui::End();
}

void GraphicsManager::cleanUp() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    window.reset();  // Explicitly destroy window before terminating GLFW
    glfwTerminate();
}

void GraphicsManager::initializeImGui() {
    glfwMakeContextCurrent(window.get());
    ImGui::CreateContext();
    ImGui_ImplGlfw_InitForOpenGL(window.get(), true);
    ImGui_ImplOpenGL3_Init("#version 330");
}

