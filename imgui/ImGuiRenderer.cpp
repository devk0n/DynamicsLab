#include "ImGuiManager.h"

void ImGuiManager::showRendererControls(Renderer &renderer) {
	ImGui::Begin("Renderer Controls");

	// 1. You need somewhere to store/edit the color.
	//    Often you'll read from the renderer, or store it statically.
	static float clearColor[4] = {0.1f, 0.1f, 0.1f, 1.0f};

	// 2. Provide an ImGui color edit widget
	if (ImGui::ColorEdit3("Background Color", clearColor, ImGuiColorEditFlags_NoInputs)) {
		// This block is called when the user changes the color
		renderer.setClearColor(clearColor[0], clearColor[1], clearColor[2], clearColor[3]);
	}

	ImGui::Checkbox("Wireframe Mode", &renderer.wireframeMode);

	ImGui::End();
}
