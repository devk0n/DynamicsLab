#include <iostream>
#include <vector>
#include <thread>
#include <imgui.h>

#include "graphics/graphics_manager.h"
#include "dynamics/simulation.h"

int main() {

    try {
        GraphicsManager graphicsManager;
        Simulation simulation(&graphicsManager);

        // Start the simulation thread
        std::thread simThread(&Simulation::run, &simulation);

        // Run graphics in the main thread
        graphicsManager.run();

        // Stop the simulation once graphics exit
        simulation.stop();
        simThread.join();  // Wait for the simulation thread to finish

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

/**
*
void mainLoop(GLFWwindow* window) {
    constexpr double timeStep = 0.001;  // 1 ms simulation timestep
    constexpr double totalTime = 100.0;  // 10 seconds simulation

    VectorXd q1(7);
    q1 << 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
    VectorXd q1d(7);
    q1d.setZero();

    std::vector<float> timeData;
    std::vector<float> posXData, posYData, posZData;
    double elapsedSimulationTime = 0.0;

    auto lastTime = std::chrono::high_resolution_clock::now();
    double realTimeElapsed = 0.0;

    while (!glfwWindowShouldClose(window)) {
        auto currentTime = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> deltaTime = currentTime - lastTime;
        lastTime = currentTime;
        realTimeElapsed += deltaTime.count();

        // Run simulation step to match real-time execution
        if (realTimeElapsed >= timeStep && elapsedSimulationTime < totalTime) {
            stepSimulation(timeStep, q1, q1d);
            elapsedSimulationTime += timeStep;

            // Store the position values over time
            timeData.push_back(static_cast<float>(elapsedSimulationTime));
            posXData.push_back(static_cast<float>(q1(0)));
            posYData.push_back(static_cast<float>(q1(1)));
            posZData.push_back(static_cast<float>(q1(2)));

            realTimeElapsed -= timeStep;
        }

        // Begin new ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("Simulation Monitoring");

        // Show the current position values
        ImGui::Text("Simulation Time: %.2f s", elapsedSimulationTime);
        ImGui::Text("Current Position X: %.3f", q1(0));
        ImGui::Text("Current Position Y: %.3f", q1(1));
        ImGui::Text("Current Position Z: %.3f", q1(2));

        ImGui::End();

        // Render ImGui frame
        ImGui::Render();
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
        glfwPollEvents();
    }
}
*/