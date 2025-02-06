#include "Grid.h"
#include "../ShaderUtils.h"      // We use ShaderUtils here
#include <iostream>
#include <vector>             // for std::vector

#include <glm/gtc/type_ptr.hpp>  // glm::value_ptr

Grid::Grid(int size)
    : m_VAO(0)
    , m_VBO(0)
    , m_shaderProgram(0)
    , m_size(size) {
}

Grid::~Grid() {
    shutdown();
}

bool Grid::initialize() {
    // Provide your real shader paths here
    const std::string vertexPath   = "../src/graphics/shaders/grid.vert";
    const std::string fragmentPath = "../src/graphics/shaders/grid.frag";

    // Create the shader program
    m_shaderProgram = ShaderUtils::createShaderProgram(vertexPath, fragmentPath);
    if (!m_shaderProgram) {
        std::cerr << "[Grid] Failed to create shader program.\n";
        return false;
    }

    return setupGrid();
}

bool Grid::setupGrid() {
    std::vector<glm::vec3> gridVertices;
    int step = 1;

    // Generate grid lines
    for (int i = -m_size; i <= m_size; ++i) {
        gridVertices.emplace_back(i * step, 0.0f, -m_size * step);
        gridVertices.emplace_back(i * step, 0.0f,  m_size * step);
    }
    for (int j = -m_size; j <= m_size; ++j) {
        gridVertices.emplace_back(-m_size * step, 0.0f, j * step);
        gridVertices.emplace_back( m_size * step, 0.0f, j * step);
    }

    glGenVertexArrays(1, &m_VAO);
    glGenBuffers(1, &m_VBO);

    glBindVertexArray(m_VAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_VBO);

    glBufferData(GL_ARRAY_BUFFER, gridVertices.size() * sizeof(glm::vec3), gridVertices.data(), GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)nullptr);

    glBindVertexArray(0);

    if (m_VAO == 0 || m_VBO == 0) {
        std::cerr << "[Grid] ERROR: Failed to generate VAO/VBO!\n";
        return false;
    }

    return true;
}


void Grid::render(const glm::mat4& projection, const glm::mat4& view) {
    if (m_shaderProgram == 0) {
        std::cerr << "[Grid] ERROR: Shader program is not set. Skipping rendering.\n";
        return;
    }

    glUseProgram(m_shaderProgram);

    GLint projLoc = glGetUniformLocation(m_shaderProgram, "projection");
    GLint viewLoc = glGetUniformLocation(m_shaderProgram, "view");

    if (projLoc == -1 || viewLoc == -1) {
        std::cerr << "[Grid] ERROR: Uniforms 'projection' or 'view' not found in shader!\n";
        return;
    }

    glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(projection));
    glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));

    glBindVertexArray(m_VAO);

    int totalLines = (m_size * 2 + 1) * 2;
    int totalVertices = totalLines * 2;

    glDrawArrays(GL_LINES, 0, totalVertices);

    glBindVertexArray(0);
    glUseProgram(0);
}


void Grid::shutdown() {
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
