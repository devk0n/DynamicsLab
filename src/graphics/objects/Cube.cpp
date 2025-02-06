#include "Cube.h"


Cube::Cube(glm::vec3 position, glm::vec3 dimensions, glm::vec3 color, glm::quat orientation) :
    m_position(position),
    m_dimensions(dimensions),
    m_color(color),
    m_orientation(orientation),
    m_shaderProgram(0),
    m_VAO(0),
    m_VBO(0) {

    // Load shader program
    m_shaderProgram = ShaderUtils::createShaderProgram("../src/graphics/shaders/cube.vert", "../src/graphics/shaders/cube.frag");
    if (!m_shaderProgram) {
        std::cerr << "[Cube] ERROR: Failed to load shaders.\n";
    }

    setupCube(); // Generate the VAO/VBO
}

Cube::~Cube() {
    shutdown();
}

void Cube::setupCube() {
    // Cube vertex positions (6 faces, 2 triangles per face)
    static const float cubeVertices[] = {
            // Back face
            -0.5f, -0.5f, -0.5f,   // Bottom-left
             0.5f,  0.5f, -0.5f,   // Top-right
             0.5f, -0.5f, -0.5f,   // Bottom-right
             0.5f,  0.5f, -0.5f,   // Top-right
            -0.5f, -0.5f, -0.5f,   // Bottom-left
            -0.5f,  0.5f, -0.5f,   // Top-left

            // Front face
            -0.5f, -0.5f,  0.5f,   // Bottom-left
             0.5f, -0.5f,  0.5f,   // Bottom-right
             0.5f,  0.5f,  0.5f,   // Top-right
             0.5f,  0.5f,  0.5f,   // Top-right
            -0.5f,  0.5f,  0.5f,   // Top-left
            -0.5f, -0.5f,  0.5f,   // Bottom-left

            // Left face
            -0.5f,  0.5f,  0.5f,   // Top-right
            -0.5f,  0.5f, -0.5f,   // Top-left
            -0.5f, -0.5f, -0.5f,   // Bottom-left
            -0.5f, -0.5f, -0.5f,   // Bottom-left
            -0.5f, -0.5f,  0.5f,   // Bottom-right
            -0.5f,  0.5f,  0.5f,   // Top-right

            // Right face
            0.5f,  0.5f,  0.5f,   // Top-left
            0.5f, -0.5f, -0.5f,   // Bottom-right
            0.5f,  0.5f, -0.5f,   // Top-right
            0.5f, -0.5f, -0.5f,   // Bottom-right
            0.5f,  0.5f,  0.5f,   // Top-left
            0.5f, -0.5f,  0.5f,   // Bottom-left

            // Bottom face
            -0.5f, -0.5f, -0.5f,   // Top-right
             0.5f, -0.5f, -0.5f,   // Top-left
             0.5f, -0.5f,  0.5f,   // Bottom-left
             0.5f, -0.5f,  0.5f,   // Bottom-left
            -0.5f, -0.5f,  0.5f,   // Bottom-right
            -0.5f, -0.5f, -0.5f,   // Top-right

            // Top face
            -0.5f,  0.5f, -0.5f,   // Top-left
             0.5f,  0.5f,  0.5f,   // Bottom-right
             0.5f,  0.5f, -0.5f,   // Top-right
             0.5f,  0.5f,  0.5f,   // Bottom-right
            -0.5f,  0.5f, -0.5f,   // Top-left
            -0.5f,  0.5f,  0.5f    // Bottom-left
    };

    glGenVertexArrays(1, &m_VAO);
    glGenBuffers(1, &m_VBO);

    glBindVertexArray(m_VAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_VBO);

    glBufferData(GL_ARRAY_BUFFER, sizeof(cubeVertices), cubeVertices, GL_STATIC_DRAW);

    // Define vertex attributes
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*) nullptr);
    glEnableVertexAttribArray(0);

    glBindVertexArray(0);
}

// Render the Cube
void Cube::render(const glm::mat4& projection, const glm::mat4& view) const {
    if (!m_shaderProgram || m_VAO == 0) {
        std::cerr << "[Cube] ERROR: Shader Program or VAO not initialized.\n";
        return;
    }

    glUseProgram(m_shaderProgram);

    GLint modelLoc = glGetUniformLocation(m_shaderProgram, "model");
    GLint viewLoc = glGetUniformLocation(m_shaderProgram, "view");
    GLint projLoc = glGetUniformLocation(m_shaderProgram, "projection");
    GLint colorLoc = glGetUniformLocation(m_shaderProgram, "cubeColor");

    glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(getModelMatrix()));
    glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(projection));
    glUniform3fv(colorLoc, 1, glm::value_ptr(m_color));

    // Draw the box
    glBindVertexArray(m_VAO);
    glDrawArrays(GL_TRIANGLES, 0, 36);
    glBindVertexArray(0);

    glUseProgram(0);
}

// Cleanup allocated GPU resources
void Cube::shutdown() {
    if (m_VAO) {
        glDeleteVertexArrays(1, &m_VAO);
        m_VAO = 0;
    }
    if (m_VBO) {
        glDeleteBuffers(1, &m_VBO);
        m_VBO = 0;
    }
    if (m_shaderProgram) {
        glDeleteProgram(m_shaderProgram);
        m_shaderProgram = 0;
    }
}

// Calculate the model matrix for the box
glm::mat4 Cube::getModelMatrix() const {
    glm::mat4 model = glm::translate(glm::mat4(1.0f), m_position);
    model *= glm::mat4_cast(m_orientation); // Apply rotation
    model = glm::scale(model, m_dimensions); // Scale to dimensions
    return model;
}
