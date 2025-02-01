#include "imgui_layer.h"

#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "renderer.h"

using namespace Eigen;

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

void ImGuiLayer::showSimulationControls() {
    ImGui::Begin("Simulation Controls");

    if (ImGui::Button("Start Simulation")) {
        m_Dynamics->startSimulation();
    }
    ImGui::SameLine();
    if (ImGui::Button("Stop Simulation")) {
        m_Dynamics->stopSimulation();
    }

    if (ImGui::Button("Reset Simulation")) {
        m_Dynamics->resetSimulation();
    }

    ImGui::Text("Total Bodies: %zu", m_Dynamics->getBodyCount());


    static const double minStepSize = 0.0000001;
    static const double maxStepSize = 0.0001;
    static double stepSize = (minStepSize + maxStepSize) * 0.5;

    if (ImGui::Button("Step Simulation")) {
        m_Dynamics->step();
    }

    ImGui::SetNextItemWidth(150.0f);
    if (ImGui::SliderScalar("##StepSizeSlider", ImGuiDataType_Double, &stepSize, &minStepSize, &maxStepSize, "Step Size: %.7f")) {
        m_Dynamics->setStepTime(stepSize);
    }

    // Static variables to store user-selected force values
    static double forceX = 0.0;
    static double forceY = 0.0;
    static double forceZ = 0.0;
    static double torqueX = 0.0;
    static double torqueY = 0.0;
    static double torqueZ = 0.0;

    static double maxForce = 1000.0;
    static double minForce = -1000.0;
    static double maxTorque = 1000.0;
    static double minTorque = -1000.0;

    // Add sliders for each component of the force
    ImGui::Text("Set External Forces:");
    ImGui::SliderScalar("Force X", ImGuiDataType_Double, &forceX, &maxForce, &minForce, "%.1f");
    ImGui::SliderScalar("Force Y", ImGuiDataType_Double, &forceY, &maxForce, &minForce, "%.1f");
    ImGui::SliderScalar("Force Z", ImGuiDataType_Double, &forceZ, &maxForce, &minForce, "%.1f");

    ImGui::SliderScalar("Torque X", ImGuiDataType_Double, &torqueX, &maxTorque, &minTorque, "%.1f");
    ImGui::SliderScalar("Torque Y", ImGuiDataType_Double, &torqueY, &maxTorque, &minTorque, "%.1f");
    ImGui::SliderScalar("Torque Z", ImGuiDataType_Double, &torqueZ, &maxTorque, &minTorque, "%.1f");

    if (ImGui::Button("Reset Force & Torque")) {
        forceX = 0.0;
        forceY = 0.0;
        forceZ = 0.0;
        torqueX = 0.0;
        torqueY = 0.0;
        torqueZ = 0.0;
    }

    Eigen::Vector3d externalForces;
    Eigen::Vector3d externalTorques;
    externalForces;
    externalForces.setZero();
    externalForces[0] = forceX;
    externalForces[1] = forceY;
    externalForces[2] = forceZ;

    externalTorques[0] = torqueX;
    externalTorques[1] = torqueY;
    externalTorques[2] = torqueZ;

    m_Dynamics->setExternalForces(externalForces);
    m_Dynamics->setExternalTorques(externalTorques);

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

        if (ImGui::BeginTabBar("MatricesTabBar")) {

            if (ImGui::BeginTabItem("Generalized Data")) {

                if (ImGui::BeginTable("GeneralizedDataTable", 4, ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
                    // Headers
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0); ImGui::Text("Positions (q)");
                    ImGui::TableSetColumnIndex(1); ImGui::Text("Velocities (qd)");
                    ImGui::TableSetColumnIndex(2); ImGui::Text("Accelerations (qdd)");
                    ImGui::TableSetColumnIndex(3); ImGui::Text("External Forces (g*)");

                    // Get matrices
                    const MatrixXd& q   = m_Dynamics->getGeneralizedCoordinates();
                    const MatrixXd& qd  = m_Dynamics->getGeneralizedVelocities();
                    const MatrixXd& qdd = m_Dynamics->getGeneralizedAccelerations();
                    const MatrixXd& g   = m_Dynamics->getGeneralizedExternalForces();

                    // Determine max rows
                    int maxRows = std::max({q.rows(), qd.rows(), qdd.rows(), g.rows()});

                    // Display row by row
                    for (int i = 0; i < maxRows; ++i) {
                        ImGui::TableNextRow();

                        // (q)
                        ImGui::TableSetColumnIndex(0);
                        if (i < q.rows()) {
                            for (int j = 0; j < q.cols(); ++j) {
                                ImGui::Text("%.4f", q(i, j));
                                ImGui::SameLine();
                            }
                        }

                        // (qd)
                        ImGui::TableSetColumnIndex(1);
                        if (i < qd.rows()) {
                            for (int j = 0; j < qd.cols(); ++j) {
                                ImGui::Text("%.4f", qd(i, j));
                                ImGui::SameLine();
                            }
                        }

                        // (qdd)
                        ImGui::TableSetColumnIndex(2);
                        if (i < qdd.rows()) {
                            for (int j = 0; j < qdd.cols(); ++j) {
                                ImGui::Text("%.4f", qdd(i, j));
                                ImGui::SameLine();
                            }
                        }

                        // G
                        ImGui::TableSetColumnIndex(3);
                        if (i < g.rows()) {
                            for (int j = 0; j < g.cols(); ++j) {
                                ImGui::Text("%.4f", g(i, j));
                                ImGui::SameLine();
                            }
                        }
                    }
                    ImGui::EndTable();
                }
                ImGui::EndTabItem();
            }

            // Instead of storing const MatrixXd&, store by value here:
            std::vector<std::pair<const char*, MatrixXd>> matrices = {
                // {"(b*) Velocity Dependent Term",       m_Dynamics->getVelocityDependentTerm()},
                // {"(M*) System Mass Inertia Matrix",    m_Dynamics->getSystemMassInertiaMatrix()},
                // {"(P) Quaternion Constraint Matrix",   m_Dynamics->getQuaternionConstraintMatrix()},
                // {"(c) Quaternion Norm Squared",        m_Dynamics->getQuaternionNormSquared()},
                {"Matrix A",                           m_Dynamics->getMatrixA()},
                {"Matrix B",                           m_Dynamics->getMatrixB()},
                {"Matrix X",                           m_Dynamics->getMatrixX()},
            };

            // Loop over the matrices
            for (auto& matrixInfo : matrices) {
                if (ImGui::BeginTabItem(matrixInfo.first)) {
                    const auto& matrix = matrixInfo.second;  // safe reference to our local copy

                    if (matrix.size() == 0) {
                        ImGui::Text("%s is empty", matrixInfo.first);
                    } else {
                        ImGui::Text("%s: %ld x %ld", matrixInfo.first, matrix.rows(), matrix.cols());

                        if (ImGui::BeginTable(matrixInfo.first, matrix.cols(), ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
                            for (int i = 0; i < matrix.rows(); ++i) {
                                ImGui::TableNextRow();
                                for (int j = 0; j < matrix.cols(); ++j) {
                                    ImGui::TableSetColumnIndex(j);
                                    ImGui::Text("%.0f", matrix(i, j));
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




