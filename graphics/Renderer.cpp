#include "Renderer.h"

void Renderer::initialize() {
    // Enable depth testing
    glEnable(GL_DEPTH_TEST);
}

void Renderer::setClearColor(float r, float g, float b, float a) {
    m_clearColor[0] = r;
    m_clearColor[1] = g;
    m_clearColor[2] = b;
    m_clearColor[3] = a;
}

void Renderer::beginFrame() {
    // Tell OpenGL what color to clear the screen with
    glClearColor(m_clearColor[0], m_clearColor[1], m_clearColor[2], m_clearColor[3]);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}
/*
void Renderer::render(const std::vector<RigidBody>& rigidBodies, const glm::mat4& view, const glm::mat4& projection) {
    // Use the shader program
    m_shader.use();
    m_shader.setMat4("view", view);
    m_shader.setMat4("projection", projection);

    // Render each rigid body
    for (const auto& body : rigidBodies) {
        m_shader.setMat4("model", body.getModelMatrix());
        body.getMesh().draw(m_shader);
    }
}
*/
void Renderer::endFrame() {
    // Post-draw cleanup
}

void Renderer::shutdown() {
    // Clean up shaders and other resources
    // m_shader.cleanup();
}

