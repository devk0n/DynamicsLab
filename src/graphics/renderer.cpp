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
#include "tools.h"

using namespace Eigen;

// Each face of the cube has 4 vertices, each with 3 position and 3 normal values.
std::vector<double> vertices = {
    // Back face
    -0.5, -0.5, -0.5,   0.0,  0.0, -1.0,
     0.5, -0.5, -0.5,   0.0,  0.0, -1.0,
     0.5,  0.5, -0.5,   0.0,  0.0, -1.0,
    -0.5,  0.5, -0.5,   0.0,  0.0, -1.0,
    // Front face
    -0.5, -0.5,  0.5,   0.0,  0.0,  1.0,
     0.5, -0.5,  0.5,   0.0,  0.0,  1.0,
     0.5,  0.5,  0.5,   0.0,  0.0,  1.0,
    -0.5,  0.5,  0.5,   0.0,  0.0,  1.0,
    // Left face
    -0.5, -0.5, -0.5,  -1.0,  0.0,  0.0,
    -0.5,  0.5, -0.5,  -1.0,  0.0,  0.0,
    -0.5,  0.5,  0.5,  -1.0,  0.0,  0.0,
    -0.5, -0.5,  0.5,  -1.0,  0.0,  0.0,
    // Right face
     0.5, -0.5, -0.5,   1.0,  0.0,  0.0,
     0.5,  0.5, -0.5,   1.0,  0.0,  0.0,
     0.5,  0.5,  0.5,   1.0,  0.0,  0.0,
     0.5, -0.5,  0.5,   1.0,  0.0,  0.0,
    // Bottom face
    -0.5, -0.5, -0.5,   0.0, -1.0,  0.0,
     0.5, -0.5, -0.5,   0.0, -1.0,  0.0,
     0.5, -0.5,  0.5,   0.0, -1.0,  0.0,
    -0.5, -0.5,  0.5,   0.0, -1.0,  0.0,
    // Top face
    -0.5,  0.5, -0.5,   0.0,  1.0,  0.0,
     0.5,  0.5, -0.5,   0.0,  1.0,  0.0,
     0.5,  0.5,  0.5,   0.0,  1.0,  0.0,
    -0.5,  0.5,  0.5,   0.0,  1.0,  0.0,
};

// Indices for the 12 triangles (6 faces)
std::vector<GLuint> indices = {
    0,  1,  2,  2,  3,  0,        // back face
    4,  5,  6,  6,  7,  4,        // front face
    8,  9, 10, 10, 11,  8,        // left face
   12, 13, 14, 14, 15, 12,        // right face
   16, 17, 18, 18, 19, 16,        // bottom face
   20, 21, 22, 22, 23, 20         // top face
};


Renderer::Renderer(GLFWwindow* window)
    : m_Window(window) {

    if (!m_Window) {
        throw std::runtime_error("Renderer: Invalid GLFW window.");
    }

    glfwMakeContextCurrent(m_Window);
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        throw std::runtime_error("Failed to initialize GLAD");
    }

    glfwGetWindowSize(m_Window, &m_width, &m_height);
    initOpenGL();

    // Set the mouse scroll callback
    glfwSetScrollCallback(m_Window, [](GLFWwindow* window, double xOffset, double yOffset) {
        auto renderer = static_cast<Renderer*>(glfwGetWindowUserPointer(window));
        if (renderer) {
            renderer->handleMouseScroll(yOffset);
        }
    });

    glfwSetWindowUserPointer(m_Window, this); // Bind this to the GLFW window pointer


    // Load grid shader (already in your code)
std::string gridVertexShaderSource = loadShaderFromFile("C:/Users/devkon/CLionProjects/DynamicsLab/assets/shaders/grid.vert.glsl");
std::string gridFragmentShaderSource = loadShaderFromFile("C:/Users/devkon/CLionProjects/DynamicsLab/assets/shaders/grid.frag.glsl");
m_GridShaderProgram = createShaderProgram(gridVertexShaderSource.c_str(), gridFragmentShaderSource.c_str());

// Load box shader (new)
std::string boxVertexShaderSource = loadShaderFromFile("C:/Users/devkon/CLionProjects/DynamicsLab/assets/shaders/box.vert.glsl");
std::string boxFragmentShaderSource = loadShaderFromFile("C:/Users/devkon/CLionProjects/DynamicsLab/assets/shaders/box.frag.glsl");
m_BoxShaderProgram = createShaderProgram(boxVertexShaderSource.c_str(), boxFragmentShaderSource.c_str());


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



    glViewport(0, 0, m_width, m_height);
    std::cout << "OpenGL Initialized" << std::endl;
}

void Renderer::clearScreen(const glm::dvec4& color) {
    glClearColor(static_cast<float>(color.r),
                 static_cast<float>(color.g),
                 static_cast<float>(color.b),
                 static_cast<float>(color.a));
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void Renderer::draw(Dynamics* dynamics) {
    // Clear the screen
    clearScreen(glm::dvec4(0.1, 0.1, 0.1, 1.0));

    glm::vec3 lightPos = glm::vec3(2.0f, 4.0f, 2.0f);
    glm::vec3 lightColor = glm::vec3(1.0f, 1.0f, 1.0f);
    glm::vec3 objectColor = glm::vec3(0.9529f, 0.2941f, 0.4902f);  // Your pinkish box

    glUseProgram(m_GridShaderProgram);
    glUniform3fv(glGetUniformLocation(m_GridShaderProgram, "lightPos"), 1, glm::value_ptr(lightPos));
    glUniform3fv(glGetUniformLocation(m_GridShaderProgram, "lightColor"), 1, glm::value_ptr(lightColor));
    glUniform3fv(glGetUniformLocation(m_GridShaderProgram, "objectColor"), 1, glm::value_ptr(objectColor));

    // Update projection and view matrices
    double aspectRatio = static_cast<float>(m_width) / static_cast<float>(m_height); // Adjust dynamically if needed
    updateProjectionMatrix(aspectRatio);
    updateViewMatrix();

    // Draw the grid
    if (m_DrawGrid) {
        drawGrid(10.0, 20, glm::dvec3(1.0, 1.0, 1.0));
    }



    for (int i = 0; i < dynamics->getBodyCount(); i++) {
        drawBox(dynamics->getBody(i)->getPosition(), Vector3d(1.0, 1.0, 1.0), dynamics->getBody(i)->getOrientation(), Vector3d(1.0, 0.2, 0.8));
    }
}

void Renderer::drawBox(const Vector3d& position, const Vector3d& scale,
                         const Vector4d& rotation, const Vector3d& color) const {
    // Convert Eigen types to glm types (using double precision)
    glm::dquat quat(rotation.w(), rotation.x(), rotation.y(), rotation.z());
    glm::dvec3 glmPosition(position.x(), position.y(), position.z());
    glm::dvec3 glmScale(scale.x(), scale.y(), scale.z());
    glm::dvec3 glmColor(color.x(), color.y(), color.z());

    // Compute the model matrix in double precision.
    glm::dmat4 model = glm::dmat4(1.0);
    model = glm::translate(model, glmPosition);
    model *= glm::mat4_cast(quat);  // Use the double-precision version!
    model = glm::scale(model, glmScale);

    // Use the box shader program.
    glUseProgram(m_BoxShaderProgram);
    glUniformMatrix4dv(glGetUniformLocation(m_BoxShaderProgram, "u_Model"), 1, GL_FALSE, glm::value_ptr(model));

    // Set view and projection matrices.
    glm::dmat4 view = glm::lookAt(m_CameraPos, m_CameraPos + m_CameraFront, m_CameraUp);
    glm::dmat4 projection = glm::perspective(glm::radians(45.0),
                                              static_cast<double>(m_width) / static_cast<double>(m_height),
                                              0.1, 100.0);
    glUniformMatrix4dv(glGetUniformLocation(m_BoxShaderProgram, "u_View"), 1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4dv(glGetUniformLocation(m_BoxShaderProgram, "u_Projection"), 1, GL_FALSE, glm::value_ptr(projection));

    // Set lighting uniforms (as floats).
    glm::vec3 lightPos(2.0f, 4.0f, 2.0f);
    glm::vec3 lightColor(1.0f, 1.0f, 1.0f);
    glm::vec3 objectColor = glm::vec3(glmColor);  // convert double to float
    glm::vec3 viewPos = glm::vec3(m_CameraPos);     // convert double to float

    glUniform3fv(glGetUniformLocation(m_BoxShaderProgram, "lightPos"), 1, glm::value_ptr(lightPos));
    glUniform3fv(glGetUniformLocation(m_BoxShaderProgram, "lightColor"), 1, glm::value_ptr(lightColor));
    glUniform3fv(glGetUniformLocation(m_BoxShaderProgram, "objectColor"), 1, glm::value_ptr(objectColor));
    glUniform3fv(glGetUniformLocation(m_BoxShaderProgram, "viewPos"), 1, glm::value_ptr(viewPos));

    // --- Setup the VAO/VBO/EBO for the box (vertex positions and normals) ---
    GLuint VAO, VBO, EBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(double), vertices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint), indices.data(), GL_STATIC_DRAW);

    // The vertex data has 6 doubles per vertex: 3 for position, 3 for normal.
    glVertexAttribPointer(0, 3, GL_DOUBLE, GL_FALSE, 6 * sizeof(double), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_DOUBLE, GL_FALSE, 6 * sizeof(double), (void*)(3 * sizeof(double)));
    glEnableVertexAttribArray(1);

    // Draw the box.
    glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(indices.size()), GL_UNSIGNED_INT, nullptr);

    // Cleanup (for this example; in a production system you might reuse buffers)
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

    // Update grid shader
    glUseProgram(m_GridShaderProgram);
    glUniformMatrix4dv(glGetUniformLocation(m_GridShaderProgram, "u_View"), 1, GL_FALSE, glm::value_ptr(view));

    // Update box shader
    glUseProgram(m_BoxShaderProgram);
    glUniformMatrix4dv(glGetUniformLocation(m_BoxShaderProgram, "u_View"), 1, GL_FALSE, glm::value_ptr(view));
}

void Renderer::updateProjectionMatrix(double aspectRatio) const {
    glm::dmat4 projection = glm::perspective(glm::radians(45.0), aspectRatio, 0.1, 100.0);

    // Update grid shader
    glUseProgram(m_GridShaderProgram);
    glUniformMatrix4dv(glGetUniformLocation(m_GridShaderProgram, "u_Projection"), 1, GL_FALSE, glm::value_ptr(projection));

    // Update box shader
    glUseProgram(m_BoxShaderProgram);
    glUniformMatrix4dv(glGetUniformLocation(m_BoxShaderProgram, "u_Projection"), 1, GL_FALSE, glm::value_ptr(projection));
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
