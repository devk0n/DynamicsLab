#include <iostream>
#include <vector>
#include <stdexcept>

#include "glad/glad.h"
#include "glm/glm.hpp"
#include <glm/gtc/matrix_transform.hpp> // For glm::lookAt
#include <glm/gtc/type_ptr.hpp>

#include "renderer.h"

Renderer::Renderer(GLFWwindow* window)
    : m_Window(window) {
if (!m_Window) {
        throw std::runtime_error("Renderer: Invalid GLFW window.");
    }

    glfwMakeContextCurrent(m_Window);
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        throw std::runtime_error("Failed to initialize GLAD");
    }

    initOpenGL();

    const char* vertexShaderSource = R"(
        #version 330 core
        layout (location = 0) in vec3 a_Position;
        uniform mat4 u_Projection;
        uniform mat4 u_View;
        void main() {
            gl_Position = u_Projection * u_View * vec4(a_Position, 1.0);
        }
    )";

    const char* fragmentShaderSource = R"(
        #version 330 core
        out vec4 FragColor;
        uniform vec3 u_Color;
        void main() {
            FragColor = vec4(u_Color, 1.0);
        }
    )";

    m_GridShaderProgram = createShaderProgram(vertexShaderSource, fragmentShaderSource);

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
    clearScreen(glm::vec4(0.1f, 0.1f, 0.1f, 1.0f));

    float aspectRatio = 1920.0f / 1080.0f; // Adjust for your window size
    updateProjectionMatrix(aspectRatio);
    updateViewMatrix();

    drawGrid(10.0f, 20, glm::vec3(1.0f, 1.0f, 1.0f));
}


void Renderer::drawGrid(float size, int divisions, const glm::vec3& color) const {
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

void Renderer::handleMouseMovement(double xpos, double ypos) {
    if (!m_RightMouseHeld) return; // Only rotate when the right button is held

    if (m_FirstMouse) {
        m_LastMouseX = xpos;
        m_LastMouseY = ypos;
        m_FirstMouse = false;
    }

    float xOffset = xpos - m_LastMouseX;
    float yOffset = m_LastMouseY - ypos; // Reversed: y-coordinates go bottom to top
    m_LastMouseX = xpos;
    m_LastMouseY = ypos;

    const float sensitivity = 0.1f; // Adjust sensitivity
    xOffset *= sensitivity;
    yOffset *= sensitivity;

    m_Yaw += xOffset;
    m_Pitch += yOffset;

    // Constrain pitch to prevent flipping
    if (m_Pitch > 89.0f)
        m_Pitch = 89.0f;
    if (m_Pitch < -89.0f)
        m_Pitch = -89.0f;

    // Update camera front vector
    glm::vec3 front;
    front.x = cos(glm::radians(m_Yaw)) * cos(glm::radians(m_Pitch));
    front.y = sin(glm::radians(m_Pitch));
    front.z = sin(glm::radians(m_Yaw)) * cos(glm::radians(m_Pitch));
    m_CameraFront = glm::normalize(front);
}

void Renderer::handleMouseButton(int button, int action) {
    if (button == GLFW_MOUSE_BUTTON_RIGHT) {
        if (action == GLFW_PRESS) {
            m_RightMouseHeld = true;
            m_FirstMouse = true; // Reset first mouse movement
            glfwSetInputMode(m_Window, GLFW_CURSOR, GLFW_CURSOR_DISABLED); // Hide cursor
        } else if (action == GLFW_RELEASE) {
            m_RightMouseHeld = false;
            glfwSetInputMode(m_Window, GLFW_CURSOR, GLFW_CURSOR_NORMAL); // Show cursor
        }
    }
}

void Renderer::updateViewMatrix() {
    glm::mat4 view = glm::lookAt(m_CameraPos, m_CameraPos + m_CameraFront, m_CameraUp);
    glUseProgram(m_GridShaderProgram);
    glUniformMatrix4fv(glGetUniformLocation(m_GridShaderProgram, "u_View"), 1, GL_FALSE, glm::value_ptr(view));
}

void Renderer::updateProjectionMatrix(float aspectRatio) {
    glm::mat4 projection = glm::perspective(glm::radians(45.0f), aspectRatio, 0.1f, 100.0f);
    glUseProgram(m_GridShaderProgram);
    glUniformMatrix4fv(glGetUniformLocation(m_GridShaderProgram, "u_Projection"), 1, GL_FALSE, glm::value_ptr(projection));
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


