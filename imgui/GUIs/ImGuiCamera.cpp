#include "../ImGuiManager.h"

void ImGuiManager::showCameraControls(const Camera &camera) {
	ImGui::Begin("Camera Controls");
	ImGui::Text("Camera Position");
	ImGui::Text("%.2f, %.2f, %.2f", camera.getPosition().x, camera.getPosition().y, camera.getPosition().z);

	ImGui::Text("Camera Front");
	ImGui::Text("%.2f, %.2f, %.2f", camera.getFront().x, camera.getFront().y, camera.getFront().z);
	// ImGui::Text("Camera Up: (%.2f, %.2f, %.2f)", camera.getUp().x, camera.getUp().y, camera.getUp().z);
	ImGui::Text("Camera Yaw: %.2f degrees", camera.getYaw());
	ImGui::Text("Camera Pitch: %.2f degrees", camera.getPitch());
	ImGui::Text("Camera Movement Speed: %.2f", camera.getMovementSpeed());
	ImGui::Text("Camera Mouse Sensitivity: %.2f", camera.getMouseSensitivity());
	ImGui::End();
}
