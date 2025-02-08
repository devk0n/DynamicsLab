#include "ImGuiManager.h"


ImGuiManager::ImGuiManager(Dynamics* dynamics) : m_dynamics(dynamics) {

}

bool ImGuiManager::initialize(GLFWwindow* window) {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();

    if (!ImGui_ImplGlfw_InitForOpenGL(window, true)) {
        std::cerr << "Error: Failed to initialize ImGui GLFW backend. \n";
        return false;
    }

    constexpr const char* glsl_version = "#version 460";
    if (!ImGui_ImplOpenGL3_Init(glsl_version)) {
        std::cerr << "Error: Failed to initialize ImGui OpenGL3 backend. \n";
        return false;
    }

    return true;
}

void ImGuiManager::renderGui() {
    beginFrame();
    controlWindow();
    dynamicsWindow();
    performanceWindow();
    endFrame();
}

void ImGuiManager::controlWindow() {
ImGui::Begin("Simulation Controls");

    if (ImGui::Button("Start Simulation")) {
        m_dynamics->start();
    }
    ImGui::SameLine();
    if (ImGui::Button("Stop Simulation")) {
        m_dynamics->stop();
    }

    if (ImGui::Button("Reset Simulation")) {
        // m_dynamics->reset();
    }

    // ImGui::Text("Total Bodies: %zu", m_dynamics->getBodyCount());


    static const double minStepSize = 0.0000001;
    static const double maxStepSize = 0.0001;
    static double stepSize = (minStepSize + maxStepSize) * 0.5;

    if (ImGui::Button("Step Simulation")) {
        m_dynamics->step();
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

    m_dynamics->setExternalForce(externalForces);
    // m_dynamics->setExternalTorques(externalTorques);

    ImGui::End();
}

void ImGuiManager::dynamicsWindow() {
    if (!m_dynamics) return;

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
                    const MatrixXd& q   = m_dynamics->getGeneralizedCoordinates();
                    const MatrixXd& qd  = m_dynamics->getGeneralizedVelocities();
                    const MatrixXd& qdd = m_dynamics->getGeneralizedAccelerations();
                    const MatrixXd& g   = m_dynamics->getGeneralizedExternalForces();

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
                // {"(b*) Velocity Dependent Term",       m_dynamics->getVelocityDependentTerm()},
                // {"(M*) System Mass Inertia Matrix",    m_dynamics->getSystemMassInertiaMatrix()},
                // {"(P) Quaternion Constraint Matrix",   m_dynamics->getQuaternionConstraintMatrix()},
                // {"(c) Quaternion Norm Squared",        m_dynamics->getQuaternionNormSquared()},
                {"Matrix A",                           m_dynamics->getMatrixA()},
                {"Matrix B",                           m_dynamics->getVectorB()},
                {"Matrix X",                           m_dynamics->getVectorX()},
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

void ImGuiManager::performanceWindow() const {
    ImGui::SetNextWindowPos({64, 64}, ImGuiCond_Once);
    ImGui::SetNextWindowSize({256, 128}, ImGuiCond_Once);

    ImGui::Begin("Performance Stats", nullptr, m_windowFlags);

    ImGuiIO& io = ImGui::GetIO();
    ImGui::Text("FPS: %.0f", io.Framerate);
    ImGui::Text("Frame Time: %.3f ms", 1000.0f / io.Framerate);

    ImGui::End();
}

void ImGuiManager::beginFrame() {
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
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



