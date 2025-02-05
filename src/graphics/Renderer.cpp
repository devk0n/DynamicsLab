#include <iostream>
#include <vector>
#include <stdexcept>

#include "glad/glad.h"

#include "glm/glm.hpp"
#include <glm/gtc/matrix_transform.hpp>

#include <glm/gtc/type_ptr.hpp>
#include <fstream>
#include <sstream>

#include "renderer.h"

Renderer::Renderer() {
    // Enable depth testing
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

    // Enable face culling (optional)
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glFrontFace(GL_CCW);

    std::cout << "Renderer initialized." << std::endl;

    std::string gridVertexShaderSource = loadShaderFromFile("C:/Users/devkon/CLionProjects/DynamicsLab/src/graphics/shaders/grid.vert.glsl");
    std::string gridFragmentShaderSource = loadShaderFromFile("C:/Users/devkon/CLionProjects/DynamicsLab/src/graphics/shaders/grid.frag.glsl");
    m_gridShaderProgram = createShaderProgram(gridVertexShaderSource.c_str(), gridFragmentShaderSource.c_str());

}

Renderer::~Renderer() {
    std::cout << "Renderer destroyed." << std::endl;
}

void Renderer::clearScreen() {
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Renderer::draw() {
    drawGrid(10.0, 10, glm::dvec3(0.0, 0.0, 3.0));
}

void Renderer::drawGrid(double size, int divisions, const glm::dvec3& color) const {
    // Identity matrix for position at the origin
    glm::dmat4 model = glm::mat4(1.0);
    glUseProgram(m_gridShaderProgram);
    glUniformMatrix4dv(
        glGetUniformLocation(m_gridShaderProgram, "u_Model"),
        1,
        GL_FALSE,
        glm::value_ptr(model)
    );

    // Generate grid vertices
    std::vector<double> vertices;
    double half = size / 2.0;
    double step = size / divisions;

    for (int i = 0; i <= divisions; ++i) {
        double offset = -half + step * i;

        // Line along X
        vertices.push_back(-half);
        vertices.push_back(0.0);
        vertices.push_back(offset);

        vertices.push_back(half);
        vertices.push_back(0.0);
        vertices.push_back(offset);

        // Line along Z
        vertices.push_back(offset);
        vertices.push_back(0.0);
        vertices.push_back(-half);

        vertices.push_back(offset);
        vertices.push_back(0.0);
        vertices.push_back(half);
    }

    // Allocate buffers
    GLuint VAO, VBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    // Upload to GPU
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);

    std::size_t bufferSize = vertices.size() * sizeof(double);
    if (bufferSize > static_cast<std::size_t>(std::numeric_limits<GLsizeiptr>::max())) {
        throw std::runtime_error("Grid buffer exceeds allowable size.");
    }

    glBufferData(
        GL_ARRAY_BUFFER,
        static_cast<GLsizeiptr>(bufferSize),
        vertices.data(),
        GL_STATIC_DRAW
    );

    glVertexAttribPointer(
        0,
        3,
        GL_DOUBLE,
        GL_FALSE,
        3 * sizeof(double),
        nullptr
    );
    glEnableVertexAttribArray(0);

    // Specify the color uniform
    glUseProgram(m_gridShaderProgram);
    glUniform3dv(glGetUniformLocation(m_gridShaderProgram, "u_Color"), 1, glm::value_ptr(color));
    glBindVertexArray(VAO);

    // Draw as lines
    std::size_t count = vertices.size() / 3;
    if (count > static_cast<std::size_t>(std::numeric_limits<GLsizei>::max())) {
        throw std::runtime_error("Too many vertices for drawing grid.");
    }

    glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(count));

    // Cleanup
    glBindVertexArray(0);
    glDeleteBuffers(1, &VBO);
    glDeleteVertexArrays(1, &VAO);
}

GLuint Renderer::compileShader(GLenum type, const char* source) {
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);

    GLint success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char infoLog[512];
        glGetShaderInfoLog(shader, 512, nullptr, infoLog);
        std::cerr << "Error: Shader compilation failed\n" << infoLog << std::endl;
    }

    return shader;
}

GLuint Renderer::createShaderProgram(const char* vertexSource, const char* fragmentSource) {
    GLuint vertexShader = compileShader(GL_VERTEX_SHADER, vertexSource);
    GLuint fragmentShader = compileShader(GL_FRAGMENT_SHADER, fragmentSource);

    GLuint program = glCreateProgram();
    glAttachShader(program, vertexShader);
    glAttachShader(program, fragmentShader);
    glLinkProgram(program);

    GLint success;
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success) {
        char infoLog[512];
        glGetProgramInfoLog(program, 512, nullptr, infoLog);
        std::cerr << "Error: Shader linking failed\n" << infoLog << std::endl;
    }

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    return program;
}

std::string Renderer::loadShaderFromFile(const std::string& filepath) {
    std::ifstream file(filepath, std::ios::in);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open shader file: " + filepath);
    }

    std::stringstream buffer;
    buffer << file.rdbuf();
    file.close();
    return buffer.str();
}
