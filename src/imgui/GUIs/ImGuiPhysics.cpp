#include "ImGuiManager.h"

void ImGuiManager::showPhysicsControl(PhysicsEngine &physicsEngine) {
	ImGui::Begin("Simulation Controls");

	if (ImGui::Button("Start Simulation")) {
		physicsEngine.start();
	}
	ImGui::SameLine();
	if (ImGui::Button("Stop Simulation")) {
		physicsEngine.stop();
	}

	if (ImGui::Button("Step Simulation")) {
		physicsEngine.step();
	}

	ImGui::SameLine();
	if (ImGui::Button("Reset Simulation")) {
		physicsEngine.reset();
	}

	// Static variables to store user-selected acceleration values
	static double accelerationX = 0.0;
	static double accelerationY = 0.0;
	static double accelerationZ = -9.81;

	static double maxForce = 300.0;
	static double minForce = -300.0;

	// Add sliders for each component of the force
	ImGui::Text("Set External Acceleration:");
	ImGui::SliderScalar("X m/s^2", ImGuiDataType_Double, &accelerationX, &minForce, &maxForce, "%.1f");
	ImGui::SliderScalar("Y m/s^2", ImGuiDataType_Double, &accelerationY, &minForce, &maxForce, "%.1f");
	ImGui::SliderScalar("Z m/s^2", ImGuiDataType_Double, &accelerationZ, &minForce, &maxForce, "%.1f");

	if (ImGui::Button("Sun gravity")) { accelerationZ = -274.0; }
	if (ImGui::Button("Mercury gravity")) { accelerationZ = -3.7; }
	if (ImGui::Button("Venus gravity")) { accelerationZ = -8.87; }
	if (ImGui::Button("Earth gravity")) { accelerationZ = -9.81; }
	if (ImGui::Button("Mars gravity")) { accelerationZ = -3.71; }
	if (ImGui::Button("Jupiter gravity")) { accelerationZ = -24.79; }
	if (ImGui::Button("Saturn gravity")) { accelerationZ = -10.44; }
	if (ImGui::Button("Uranus gravity")) { accelerationZ = -8.69; }
	if (ImGui::Button("Neptune gravity")) { accelerationZ = -11.15; }
	if (ImGui::Button("Pluto gravity")) { accelerationZ = -0.62; }

	if (ImGui::Button("Reset Force & Torque")) {
		accelerationX = 0.0;
		accelerationY = 0.0;
		accelerationZ = 0.0;
	}

	Eigen::Vector3d gravity;

	gravity.setZero();
	gravity[0] = accelerationX;
	gravity[1] = accelerationY;
	gravity[2] = accelerationZ;

	physicsEngine.setGravity(gravity);
	ImGui::End();
}
