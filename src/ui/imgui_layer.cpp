#include "imgui_layer.h"

#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "renderer.h"

ImGuiLayer::ImGuiLayer(GLFWwindow* window, Renderer* renderer, Dynamics* dynamics)
    : m_Window(window), m_Renderer(renderer), m_Dynamics(dynamics) {

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
    showDynamicsData();

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

void ImGuiLayer::showDynamicsData() {
    if (!m_Dynamics) return;

    if (ImGui::Begin("Dynamics Data")) {
        ImGui::Text("Total Bodies: %zu", m_Dynamics->getBodyCount());

        // Begin a tab bar for multiple matrices
        if (ImGui::BeginTabBar("MatricesTabBar")) {

            // Define all matrices
            struct MatrixInfo {
                const char* name;
                const Eigen::MatrixXd& matrix;
            };

            std::vector<MatrixInfo> matrices = {
                {"Velocity Dependent Term", m_Dynamics->getVelocityDependentTerm()},
                {"System Mass Inertia Matrix", m_Dynamics->getSystemMassInertiaMatrix()},
                {"Quaternion Constraint Matrix", m_Dynamics->getQuaternionConstraintMatrix()},
                {"Generalized Coordinates", m_Dynamics->getGeneralizedCoordinates()},
                {"Generalized Velocities", m_Dynamics->getGeneralizedVelocities()},
                {"Generalized Accelerations", m_Dynamics->getGeneralizedAccelerations()},
                {"Quaternion Norm Squared", m_Dynamics->getQuaternionNormSquared()},
                {"Generalized External Forces", m_Dynamics->getGeneralizedExternalForces()},
                {"Matrix A", m_Dynamics->getMatrixA()},
                {"Matrix B", m_Dynamics->getMatrixB()},
                {"Matrix X", m_Dynamics->getMatrixX()}
            };

            for (const auto& matrixInfo : matrices) {
                if (ImGui::BeginTabItem(matrixInfo.name)) {
                    const Eigen::MatrixXd& matrix = matrixInfo.matrix;

                    if (matrix.size() == 0) {
                        ImGui::Text("%s is empty", matrixInfo.name);
                    } else {
                        ImGui::Text("%s: %ld x %ld", matrixInfo.name, matrix.rows(), matrix.cols());

                        if (ImGui::BeginTable(matrixInfo.name, matrix.cols(), ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
                            for (int i = 0; i < matrix.rows(); ++i) {
                                ImGui::TableNextRow();
                                for (int j = 0; j < matrix.cols(); ++j) {
                                    ImGui::TableSetColumnIndex(j);
                                    ImGui::Text("%.2f", matrix(i, j));  // Display float values with two decimal precision
                                }
                            }
                            ImGui::EndTable();
                        }
                    }

                    ImGui::EndTabItem();
                }
            }

            ImGui::EndTabBar();
        }
    }
    ImGui::End();
}



