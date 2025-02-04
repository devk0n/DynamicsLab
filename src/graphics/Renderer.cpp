#include <glad/glad.h>

#include "Renderer.h"

Renderer::Renderer() {
    // Enable depth testing
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    // Enable face culling (optional)
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glFrontFace(GL_CCW);

    std::cout << "Renderer initialized." << std::endl;
}

Renderer::~Renderer() {
    std::cout << "Renderer destroyed." << std::endl;
}

void Renderer::clearScreen() {
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Renderer::draw() {
    // Placeholder for actual drawing logic
}
