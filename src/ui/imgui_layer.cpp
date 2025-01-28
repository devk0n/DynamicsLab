#include "imgui_layer.h"

#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "renderer.h"


ImGuiLayer::ImGuiLayer(GLFWwindow* window, Renderer* renderer)
    : m_Window(window),
      m_Renderer(renderer){
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(m_Window, true);
    ImGui_ImplOpenGL3_Init("#version 460 core");

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
    showRenderingOptions(m_Renderer);
    showDebugWindow();
    showSimulationData();

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}


void ImGuiLayer::updateCameraData(const glm::dvec3& position, const glm::dvec3& orientation, const double& speed) {
    m_CameraPosition = position;
    m_CameraOrientation = orientation;
    m_CameraSpeed = speed;
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

    static double simulationSpeed = 1.0;
    static double minSpeed = 0.0;
    static double maxSpeed = 10.0;
    ImGui::SliderScalar("Speed", ImGuiDataType_Double, &simulationSpeed, &minSpeed, &maxSpeed);

    static bool enablePhysics = true;
    ImGui::Checkbox("Enable Physics", &enablePhysics);

    ImGui::End();
}


void ImGuiLayer::showRenderingOptions(Renderer* renderer) {
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

    static bool m_DrawGrid = renderer->getDrawGrid();
    ImGui::Checkbox("Draw Grid", &m_DrawGrid);
    if (m_DrawGrid) {
        renderer->setDrawGrid(true);
    } else {
        renderer->setDrawGrid(false);
    }

    ImGui::End();
}


void ImGuiLayer::showDebugWindow() const {
    ImGui::Begin("Debug Info");

    // Basic statistics
    static int frameCount = 0;
    frameCount++;
    ImGui::Text("Frame Count: %d", frameCount);
    ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);
    ImGui::Text("Frame Time: %.3f ms", 1000.0f / ImGui::GetIO().Framerate);

    // Camera information
    ImGui::Separator();
    ImGui::Text("Camera Info:");
    ImGui::Text("Position: (%.2f, %.2f, %.2f)", m_CameraPosition.x, m_CameraPosition.y, m_CameraPosition.z);
    ImGui::Text("Orientation: (%.2f, %.2f, %.2f)", m_CameraOrientation.x, m_CameraOrientation.y, m_CameraOrientation.z);
    ImGui::Text("Camera Speed: %.2f", m_CameraSpeed);

    ImGui::End();
}



void ImGuiLayer::showSimulationData() {
    ImGui::Begin("Simulation Data");

    // Show current values
    double currentPos = m_PositionHistory[(m_CurrentIndex - 1 + historySize) % historySize];
    double currentVel = m_VelocityHistory[(m_CurrentIndex - 1 + historySize) % historySize];

    ImGui::Text("Position: %.2f m", currentPos);
    ImGui::Text("Velocity: %.2f m/s", currentVel);

    ImGui::End();
}




