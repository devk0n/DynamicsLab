#include "Renderer.h"


Renderer::Renderer() :
    m_grid(5),
    m_viewMatrix(glm::lookAt(glm::vec3(0.0f, 5.0f, 10.0f), glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f))),
    m_projectionMatrix(glm::perspective(glm::radians(45.0f), 1920.0f / 1280.0f, 0.1f, 10000.0f)) {
}

Renderer::~Renderer() { shutdown(); }

bool Renderer::initialize() {
    // Set basic GL state
    glEnable(GL_DEPTH_TEST);
    // glEnable(GL_MULTISAMPLE);
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

    for (const auto& cube : m_cubes) {
        cube->render(m_projectionMatrix, m_viewMatrix);
    }
}

void Renderer::shutdown() { m_grid.shutdown(); }

void Renderer::addCube(std::unique_ptr<Cube> cube) { m_cubes.push_back(std::move(cube)); }

void Renderer::setProjectionMatrix(const glm::mat4& projection) { m_projectionMatrix = projection; }

void Renderer::setViewMatrix(const glm::mat4& view) { m_viewMatrix = view; }
