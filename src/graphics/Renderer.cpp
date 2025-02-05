//
// Created by devkon on 05/02/2025.
//

#include "Renderer.h"

bool Renderer::initialize() {
    // Configure OpenGL state
    glEnable(GL_DEPTH_TEST);

    // Set clear color (change to any RGBA you prefer)
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);

    // If there was more required setup (creating buffers, shaders, etc.), include it here

    return true; // Return false if something fails
}

void Renderer::clear() {
    // Clear color and depth buffer
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Renderer::render() {
    // Perform drawing calls here, for example:
    // glBindVertexArray(someVAO);
    // glDrawArrays(GL_TRIANGLES, 0, 36);

    // Replace with your own drawing/scene rendering logic
}
