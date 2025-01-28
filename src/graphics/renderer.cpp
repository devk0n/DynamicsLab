#include <iostream>
#include <vector>
#include <stdexcept>

#include "glad/glad.h"

#include "renderer.h"

Renderer::Renderer(GLFWwindow* window)
    : m_Window(window) {

    if (!m_Window) {
        throw std::runtime_error("Renderer: Invalid GLFW window.");
    }

    glfwMakeContextCurrent(m_Window);
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "GLAD initialization failed: Unable to load OpenGL functions." << std::endl;
        throw std::runtime_error("Failed to initialize GLAD");
    }

    initOpenGL();
}

Renderer::~Renderer() {
    std::cout << "Renderer destroyed." << std::endl;
}

void Renderer::initOpenGL() {
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) {
        throw std::runtime_error("OpenGL initialization error");
    }

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glViewport(0, 0, 1920, 1080);

    std::cout << "OpenGL Initialized" << std::endl;
}

void Renderer::clearScreen(const glm::vec4& color) {
    glClearColor(color.r, color.g, color.b, color.a);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Renderer::draw() {
    clearScreen(glm::vec4(0.1f, 0.1f, 0.1f, 1.0f)); // Background color
    drawGrid(10.0f, 20, glm::vec3(1.0f, 1.0f, 1.0f)); // White grid
}

void Renderer::drawGrid(float size, int divisions, const glm::vec3& color) {
    std::vector<float> vertices;
    float half = size / 2.0f;             // Half grid size
    float step = size / divisions;        // Distance between grid lines

    // Generate grid lines along X and Z axes
    for (int i = 0; i <= divisions; ++i) {
        float offset = -half + step * i;

        // Add two lines per iteration: one parallel to X and one parallel to Z
        // Line parallel to X-axis
        vertices.push_back(-half);    // Start point (x)
        vertices.push_back(0.0f);     // Start point (y)
        vertices.push_back(offset);   // Start point (z)
        vertices.push_back(half);     // End point (x)
        vertices.push_back(0.0f);     // End point (y)
        vertices.push_back(offset);   // End point (z)

        // Line parallel to Z-axis
        vertices.push_back(offset);   // Start point (x)
        vertices.push_back(0.0f);     // Start point (y)
        vertices.push_back(-half);    // Start point (z)
        vertices.push_back(offset);   // End point (x)
        vertices.push_back(0.0f);     // End point (y)
        vertices.push_back(half);     // End point (z)
    }

    GLuint VAO, VBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);

    // Set up vertex attributes
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // Set grid color
    glUseProgram(m_GridShaderProgram); // Assumes a shader program is created and linked
    glUniform3fv(glGetUniformLocation(m_GridShaderProgram, "u_Color"), 1, &color[0]);

    // Draw the grid
    glBindVertexArray(VAO);
    glDrawArrays(GL_LINES, 0, vertices.size() / 3);

    // Unbind and clean up
    glBindVertexArray(0);
    glDeleteBuffers(1, &VBO);
    glDeleteVertexArrays(1, &VAO);
}