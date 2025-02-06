#include "Renderer.h"   // Corresponding header
#include <iostream>      // For std::cout, std::cerr

Renderer::Renderer() :
    m_grid(10),
    m_viewMatrix(1.0f) {

    m_projectionMatrix = glm::perspective(glm::radians(45.0f), 1920.0f / 1280.0f, 0.1f, 100.0f);
    m_viewMatrix = glm::lookAt(glm::vec3(0.0f, 5.0f, 10.0f), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f));

}

Renderer::~Renderer() {
    shutdown();
}

bool Renderer::initialize() {
    // Set basic GL state
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);

    if (!m_grid.initialize()) {
        std::cerr << "[Renderer] Failed to initialize Grid.\n";
        return false;
    }

    std::cout << "[Renderer] Initialized successfully.\n";
    return true;
}

void Renderer::clear() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Renderer::render() {
    m_grid.render(m_projectionMatrix, m_viewMatrix);
}

void Renderer::shutdown() {
    m_grid.shutdown();  // No need to delete anything
}

void Renderer::setProjectionMatrix(const glm::mat4& projection) {
    m_projectionMatrix = projection;
}

void Renderer::setViewMatrix(const glm::mat4& view) {
    m_viewMatrix = view;
}
