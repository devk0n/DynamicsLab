#include "Renderer.h"
#include <fstream>
#include <sstream>
#include <vector>
#include <iostream>

Renderer::Renderer() :
    m_gridVAO(0),
    m_gridVBO(0),
    m_gridShaderProgram(0),
    m_viewMatrix(1.0f) {
}

Renderer::~Renderer() {
    shutdown();
}

bool Renderer::initialize() {
    // Set some basic OpenGL state
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);

    if (!initializeGrid()) {
        return false;
    }

    std::cout << "[Renderer] Initialized successfully.\n";
    return true;
}

void Renderer::clear() {
    // Clear color and depth buffer
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Renderer::render() {
    // Use our grid shader
    glUseProgram(m_gridShaderProgram);

    // Send the matrices to the shader
    GLint projLoc = glGetUniformLocation(m_gridShaderProgram, "projection");
    GLint viewLoc = glGetUniformLocation(m_gridShaderProgram, "view");

    glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(m_projectionMatrix));
    glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(m_viewMatrix));

    glBindVertexArray(m_gridVAO);

    // For gridSize=10, we have 21 lines each way => 42 lines => 84 vertices
    int totalLines = (10 * 2 + 1) * 2;
    int totalVertices = totalLines * 2;
    glDrawArrays(GL_LINES, 0, totalVertices);

    glBindVertexArray(0);
    glUseProgram(0);
}

void Renderer::shutdown() {
    if (m_gridVAO) {
        glDeleteVertexArrays(1, &m_gridVAO);
        m_gridVAO = 0;
    }
    if (m_gridVBO) {
        glDeleteBuffers(1, &m_gridVBO);
        m_gridVBO = 0;
    }
    if (m_gridShaderProgram) {
        glDeleteProgram(m_gridShaderProgram);
        m_gridShaderProgram = 0;
    }
}

bool Renderer::setupGrid() {
    // Build a list of lines in the X-Z plane (y=0)
    std::vector<glm::vec3> gridVertices;
    const int gridSize = 5;
    const int step = 1;

    // Vertical lines
    for (int i = -gridSize; i <= gridSize; ++i) {
        gridVertices.emplace_back(i * step, 0.0f, -gridSize * step);
        gridVertices.emplace_back(i * step, 0.0f,  gridSize * step);
    }

    // Horizontal lines
    for (int j = -gridSize; j <= gridSize; ++j) {
        gridVertices.emplace_back(-gridSize * step, 0.0f, j * step);
        gridVertices.emplace_back( gridSize * step, 0.0f, j * step);
    }

    // Create and fill buffers
    glGenVertexArrays(1, &m_gridVAO);
    glGenBuffers(1, &m_gridVBO);

    glBindVertexArray(m_gridVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_gridVBO);

    glBufferData(GL_ARRAY_BUFFER,
                 gridVertices.size() * sizeof(glm::vec3),
                 gridVertices.data(),
                 GL_STATIC_DRAW);

    // Position attribute
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(
        0,
        3,
        GL_FLOAT,
        GL_FALSE,
        sizeof(glm::vec3),
        (void*)nullptr
    );

    glBindVertexArray(0);
    return true;
}

void Renderer::setProjectionMatrix(const glm::mat4& projection) {
    m_projectionMatrix = projection;
}

void Renderer::setViewMatrix(const glm::mat4& view) {
    m_viewMatrix = view;
}

// ----------- SHADER LOADING UTILS ------------
std::string Renderer::readFileContents(const std::string& filePath) {
    std::ifstream file(filePath, std::ios::in | std::ios::binary);
    if (!file) {
        std::cerr << "[Renderer::readFileContents] Cannot open file: " << filePath << "\n";
        return {};
    }
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

static GLuint compileShader(GLenum shaderType, const std::string& source) {
    GLuint shader = glCreateShader(shaderType);
    const char* srcPtr = source.c_str();
    glShaderSource(shader, 1, &srcPtr, nullptr);
    glCompileShader(shader);

    // Check compile
    GLint success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        GLchar infoLog[1024];
        glGetShaderInfoLog(shader, 1024, nullptr, infoLog);
        std::cerr << "[compileShader] Error:\n" << infoLog << std::endl;
        glDeleteShader(shader);
        return false;
    }
    return shader;
}

GLuint Renderer::createShaderProgramFromFiles(const std::string& vertexFilePath,
                                              const std::string& fragmentFilePath) {
    // Read the files
    std::string vertexCode = readFileContents(vertexFilePath);
    std::string fragmentCode = readFileContents(fragmentFilePath);

    if (vertexCode.empty() || fragmentCode.empty()) {
        std::cerr << "[createShaderProgramFromFiles] Empty shader file.\n";
        return false;
    }

    // Compile each shader
    GLuint vertexShader   = compileShader(GL_VERTEX_SHADER, vertexCode);
    if (!vertexShader) return false;

    GLuint fragmentShader = compileShader(GL_FRAGMENT_SHADER, fragmentCode);
    if (!fragmentShader) {
        glDeleteShader(vertexShader);
        return false;
    }

    // Link them into a program
    GLuint program = glCreateProgram();
    glAttachShader(program, vertexShader);
    glAttachShader(program, fragmentShader);
    glLinkProgram(program);

    // Check linking
    GLint success;
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (!success) {
        GLchar infoLog[1024];
        glGetProgramInfoLog(program, 1024, nullptr, infoLog);
        std::cerr << "[createShaderProgramFromFiles] Link error:\n" << infoLog << std::endl;
        glDeleteShader(vertexShader);
        glDeleteShader(fragmentShader);
        glDeleteProgram(program);
        return false;
    }

    // Cleanup the shader objects; the program has them now
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    return program;
}

bool Renderer::initializeGrid() {
    // Hard-code your shader file paths:
    const std::string vertexPath = "C:/Users/devkon/CLionProjects/DynamicsLab/src/graphics/shaders/grid.vert";
    const std::string fragmentPath = "C:/Users/devkon/CLionProjects/DynamicsLab/src/graphics/shaders/grid.frag";

    // Compile/link the shader program from these files
    m_gridShaderProgram = createShaderProgramFromFiles(vertexPath, fragmentPath);
    if (!m_gridShaderProgram) {
        std::cerr << "[Renderer] Failed to create grid shader program.\n";
        return false;
    }

    // Set up the grid geometry (lines on the floor)
    if (!setupGrid()) {
        std::cerr << "[Renderer] Failed to setup grid geometry.\n";
        return false;
    }
    return true;
}
