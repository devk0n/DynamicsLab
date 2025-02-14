#include "../ImGuiManager.h"

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
		// physicsEngine.reset();
	}

	// Static variables to store user-selected force values
	static double forceX = 0.0;
	static double forceY = 0.0;
	static double forceZ = -9.81;

	static double maxForce = 300.0;
	static double minForce = -300.0;

	// Add sliders for each component of the force
	ImGui::Text("Set External Acceleration:");
	ImGui::SliderScalar("X m/s^2", ImGuiDataType_Double, &forceX, &minForce, &maxForce, "%.1f");
	ImGui::SliderScalar("Y m/s^2", ImGuiDataType_Double, &forceY, &minForce, &maxForce, "%.1f");
	ImGui::SliderScalar("Z m/s^2", ImGuiDataType_Double, &forceZ, &minForce, &maxForce, "%.1f");

	if (ImGui::Button("Reset Force & Torque")) {
		forceX = 0.0;
		forceY = 0.0;
		forceZ = 0.0;
	}

	Eigen::Vector3d gravity;

	gravity.setZero();
	gravity[0] = forceX;
	gravity[1] = forceY;
	gravity[2] = forceZ;


	if (ImGui::Button("Earth gravity")) {
		gravity[2] = -9.81;
	}

	physicsEngine.setGravity(gravity);
	ImGui::End();
}
