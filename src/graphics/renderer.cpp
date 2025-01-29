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

    // Set the mouse scroll callback
    glfwSetScrollCallback(m_Window, [](GLFWwindow* window, double xOffset, double yOffset) {
        auto renderer = static_cast<Renderer*>(glfwGetWindowUserPointer(window));
        if (renderer) {
            renderer->handleMouseScroll(yOffset);
        }
    });

    glfwSetWindowUserPointer(m_Window, this); // Bind this to the GLFW window pointer


    std::string vertexShaderSource = loadShaderFromFile("C:/Users/devkon/CLionProjects/DynamicsLab/assets/shaders/grid.vert.glsl");
    std::string fragmentShaderSource = loadShaderFromFile("C:/Users/devkon/CLionProjects/DynamicsLab/assets/shaders/grid.frag.glsl");

    m_GridShaderProgram = createShaderProgram(vertexShaderSource.c_str(), fragmentShaderSource.c_str());

}

Renderer::~Renderer() {
    std::cout << "Renderer destroyed." << std::endl;
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

void Renderer::initOpenGL() {
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) {
        throw std::runtime_error("OpenGL initialization error");
    }

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Enable Multi-Sampling
    glEnable(GL_MULTISAMPLE);

    glViewport(0, 0, 1920, 1080);
    std::cout << "OpenGL Initialized" << std::endl;
}

void Renderer::clearScreen(const glm::dvec4& color) {
    glClearColor(static_cast<float>(color.r),
                 static_cast<float>(color.g),
                 static_cast<float>(color.b),
                 static_cast<float>(color.a));
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Renderer::draw() {
    // Clear the screen
    clearScreen(glm::dvec4(0.1, 0.1, 0.1, 1.0));

    // Calculate the aspect ratio
    double aspectRatio = 1920.0 / 1080.0; // Adjust based on the window size

    // Update projection and view matrices for the scene
    updateProjectionMatrix(aspectRatio);
    updateViewMatrix();

    // Draw the grid, should not have any transformations applied (uses identity model matrix)
    if (m_DrawGrid) {
        drawGrid(10.0, 20, glm::dvec3(1.0, 1.0, 1.0));
    }


    // Draw the box, with transformations (position, rotation, scaling)
    drawBox(glm::dvec3(-1.0, 2.0, 0.0),  // Position
            glm::dvec3(2.0, 0.2, 0.2),    // Scale
            glm::dvec3(0.0, 0.0, 0.0),  // Rotation (x, y, z)
            glm::dvec3(1.0, 0.0, 0.0));   // Color


    drawBox(glm::dvec3(-3.0, 2.0, 0.0),  // Position
            glm::dvec3(2.0, 0.2, 0.2),    // Scale
            glm::dvec3(0.0, 40.0, 0.0),  // Rotation (x, y, z)
            glm::dvec3(0.0, 1.0, 0.0));   // Color
}

void Renderer::drawBox(const glm::dvec3& position, const glm::dvec3& scale, const glm::dvec3& rotation, const glm::dvec3& color) const {
    // Compute the model matrix
    glm::dmat4 model = glm::mat4(1.0); // Start with the identity matrix
    model = glm::translate(model, position); // Translate to position
    model = glm::rotate(model, glm::radians(rotation.x), glm::dvec3(1.0, 0.0, 0.0)); // Rotate around X-axis
    model = glm::rotate(model, glm::radians(rotation.y), glm::dvec3(0.0, 1.0, 0.0)); // Rotate around Y-axis
    model = glm::rotate(model, glm::radians(rotation.z), glm::dvec3(0.0, 0.0, 1.0)); // Rotate around Z-axis
    model = glm::scale(model, scale); // Scale the box

    // Pass the model matrix to the shader
    glUseProgram(m_GridShaderProgram);
    glUniformMatrix4dv(glGetUniformLocation(m_GridShaderProgram, "u_Model"), 1, GL_FALSE, glm::value_ptr(model));


    // Define vertices for the box (centered at 0,0,0)
    std::vector<double> vertices = {
        -0.5, -0.5, -0.5,   // 0
         0.5, -0.5, -0.5,   // 1
        -0.5,  0.5, -0.5,   // 2
         0.5,  0.5, -0.5,   // 3
        -0.5, -0.5,  0.5,   // 4
         0.5, -0.5,  0.5,   // 5
        -0.5,  0.5,  0.5,   // 6
         0.5,  0.5,  0.5    // 7
    };

    // Define indices (same as previously)
    std::vector<GLuint> indices = {
        0, 1,  1, 3,  3, 2,  2, 0, // Bottom face
        4, 5,  5, 7,  7, 6,  6, 4, // Top face
        0, 4,  1, 5,  2, 6,  3, 7  // Vertical edges
    };

    // Create VAO and VBO (similar as in the existing code)
    GLuint VAO, VBO, EBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    std::size_t bufferSize = vertices.size() * sizeof(double);
    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(bufferSize), vertices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(indices.size() * sizeof(GLuint)), indices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_DOUBLE, GL_FALSE, 3 * sizeof(double), nullptr);
    glEnableVertexAttribArray(0);

    // Draw the box
    glUseProgram(m_GridShaderProgram);
    glUniform3dv(glGetUniformLocation(m_GridShaderProgram, "u_Color"), 1, glm::value_ptr(color));
    glBindVertexArray(VAO);

    glDrawElements(GL_LINES, static_cast<GLsizei>(indices.size()), GL_UNSIGNED_INT, nullptr);

    // Cleanup
    glBindVertexArray(0);
    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &EBO);
    glDeleteVertexArrays(1, &VAO);
}

void Renderer::drawGrid(double size, int divisions, const glm::dvec3& color) const {
    // Set the model matrix to identity, so no transformations are applied to the grid
    glm::dmat4 model = glm::mat4(1.0); // Identity matrix
    glUseProgram(m_GridShaderProgram);
    glUniformMatrix4dv(glGetUniformLocation(m_GridShaderProgram, "u_Model"), 1, GL_FALSE, glm::value_ptr(model));

    // Generate grid vertices
    std::vector<double> vertices;
    double half = size / 2.0;
    double step = size / divisions;

    for (int i = 0; i <= divisions; ++i) {
        double offset = -half + step * i;

        vertices.push_back(-half); vertices.push_back(0.0); vertices.push_back(offset);
        vertices.push_back(half);  vertices.push_back(0.0); vertices.push_back(offset);

        vertices.push_back(offset); vertices.push_back(0.0); vertices.push_back(-half);
        vertices.push_back(offset); vertices.push_back(0.0); vertices.push_back(half);
    }

    // Create VAO and VBO
    GLuint VAO, VBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);

    std::size_t bufferSize = vertices.size() * sizeof(double);
    if (bufferSize > static_cast<std::size_t>(std::numeric_limits<GLsizeiptr>::max())) {
        throw std::runtime_error("Buffer size exceeds maximum allowable GLsizeiptr value.");
    }

    glBufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(bufferSize), vertices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_DOUBLE, GL_FALSE, 3 * sizeof(double), nullptr);
    glEnableVertexAttribArray(0);

    // Use the shader and specify the color
    glUseProgram(m_GridShaderProgram);
    glUniform3dv(glGetUniformLocation(m_GridShaderProgram, "u_Color"), 1, glm::value_ptr(color));
    glBindVertexArray(VAO);

    // Draw the grid lines
    std::size_t count = vertices.size() / 3;
    if (count > static_cast<std::size_t>(std::numeric_limits<GLsizei>::max())) {
        throw std::runtime_error("Too many vertices to draw; exceeds maximum GLsizei value.");
    }

    glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(count));

    // Cleanup
    glBindVertexArray(0);
    glDeleteBuffers(1, &VBO);
    glDeleteVertexArrays(1, &VAO);
}

void Renderer::handleMouseMovement(double xpos, double ypos) {
    if (!m_RightMouseHeld) return; // Process only if the right mouse button is held

    // On the first mouse movement, initialize the last mouse position
    if (m_FirstMouse) {
        m_LastMouseX = xpos;
        m_LastMouseY = ypos;
        m_FirstMouse = false;

        return; // Skip offset calculation on the first frame
    }

    // Calculate mouse movement offset
    double xOffset = xpos - m_LastMouseX;
    double yOffset = m_LastMouseY - ypos; // Inverted because y-coordinates go bottom to top
    m_LastMouseX = xpos;
    m_LastMouseY = ypos;

    // Apply sensitivity to mouse movement
    const double sensitivity = 0.1;
    xOffset *= sensitivity;
    yOffset *= sensitivity;

    // Update yaw and pitch
    m_Yaw += xOffset;
    m_Pitch += yOffset;

    // Constrain pitch to prevent flipping
    if (m_Pitch > 89.0) m_Pitch = 89.0;
    if (m_Pitch < -89.0) m_Pitch = -89.0;

    // Update camera's front vector based on yaw and pitch
    glm::dvec3 front;
    front.x = cos(glm::radians(m_Yaw)) * cos(glm::radians(m_Pitch));
    front.y = sin(glm::radians(m_Pitch));
    front.z = sin(glm::radians(m_Yaw)) * cos(glm::radians(m_Pitch));
    m_CameraFront = glm::normalize(front);

}

void Renderer::handleMouseButton(int button, int action) {
    if (button == GLFW_MOUSE_BUTTON_RIGHT) {
        if (action == GLFW_PRESS) {
            m_RightMouseHeld = true;
            m_FirstMouse = true; // Reset the first mouse movement flag
            glfwSetInputMode(m_Window, GLFW_CURSOR, GLFW_CURSOR_DISABLED); // Hide and lock cursor
        } else if (action == GLFW_RELEASE) {
            m_RightMouseHeld = false;

            // Get the window dimensions to calculate the center
            int width, height;
            glfwGetWindowSize(m_Window, &width, &height);
            double centerX = width / 2.0;
            double centerY = height / 2.0;

            // Reset the cursor to the center of the window
            glfwSetCursorPos(m_Window, centerX, centerY);

            glfwSetInputMode(m_Window, GLFW_CURSOR, GLFW_CURSOR_NORMAL); // Show and unlock cursor
        }
    }
}

// Method to handle mouse scroll input
void Renderer::handleMouseScroll(double yOffset) {
    const double scrollSensitivity = 0.1; // Adjust sensitivity as needed
    m_CameraSpeed += yOffset * scrollSensitivity;

    // Clamp the camera speed to avoid invalid values
    if (m_CameraSpeed < 0.1) m_CameraSpeed = 0.1; // Set minimum camera speed
    if (m_CameraSpeed > 20.0) m_CameraSpeed = 20.0; // Set maximum camera speed
}

void Renderer::handleKeyboardInput(GLFWwindow *window, double deltaTime) {
    double velocity = m_CameraSpeed * deltaTime;

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
        m_CameraPos += m_CameraFront * velocity; // Move forward
    }
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
        m_CameraPos -= m_CameraFront * velocity; // Move backward
    }
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
        m_CameraPos -= glm::normalize(glm::cross(m_CameraFront, m_CameraUp)) * velocity; // Move left
    }
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
        m_CameraPos += glm::normalize(glm::cross(m_CameraFront, m_CameraUp)) * velocity; // Move right
    }
    if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS) {
        m_CameraPos += glm::normalize(glm::cross(glm::cross(m_CameraFront, m_CameraUp), m_CameraFront)) * velocity; // Move up
    }
    if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS) {
        m_CameraPos -= glm::normalize(glm::cross(glm::cross(m_CameraFront, m_CameraUp), m_CameraFront)) * velocity; // Move down
    }
}

void Renderer::updateViewMatrix() {
    glm::dmat4 view = glm::lookAt(m_CameraPos, m_CameraPos + m_CameraFront, m_CameraUp);
    glUseProgram(m_GridShaderProgram);
    glUniformMatrix4dv(glGetUniformLocation(m_GridShaderProgram, "u_View"), 1, GL_FALSE, glm::value_ptr(view));
}

void Renderer::updateProjectionMatrix(double aspectRatio) const {
    glm::dmat4 projection = glm::perspective(glm::radians(45.0), aspectRatio, 0.1, 100.0);
    glUseProgram(m_GridShaderProgram);
    glUniformMatrix4dv(glGetUniformLocation(m_GridShaderProgram, "u_Projection"), 1, GL_FALSE, glm::value_ptr(projection));
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

glm::dvec3 Renderer::getCameraPosition() {
    return m_CameraPos;
}

glm::dvec3 Renderer::getCameraOrientation() {
    return m_CameraFront;
}

bool Renderer::getDrawGrid() const {
    return m_DrawGrid;
}

double Renderer::getCameraSpeed() const {
    return m_CameraSpeed;
}

void Renderer::setDrawGrid(bool drawGrid) {
    m_DrawGrid = drawGrid;
}
