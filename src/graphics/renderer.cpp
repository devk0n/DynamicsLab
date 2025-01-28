#include <iostream>
#include <vector>
#include <stdexcept>

#include "glad/glad.h"

#include "glm/glm.hpp"
#include <glm/gtc/matrix_transform.hpp>
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
        #version 460 core
        layout (location = 0) in dvec3 a_Position; // Input double-precision data
        uniform dmat4 u_Projection;               // Double-precision uniform
        uniform dmat4 u_View;                     // Double-precision uniform

        void main() {
            // Convert double-precision to single-precision for gl_Position
            gl_Position = mat4(u_Projection) * mat4(u_View) * vec4(a_Position, 1.0);
        }
    )";

    const char* fragmentShaderSource = R"(
        #version 460 core
        out vec4 FragColor;    // Fragment color output (single-precision)
        uniform dvec3 u_Color; // Double-precision uniform

        void main() {
            // Convert double-precision to single-precision for output
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


void Renderer::clearScreen(const glm::dvec4& color) {
    glClearColor(static_cast<float>(color.r),
                 static_cast<float>(color.g),
                 static_cast<float>(color.b),
                 static_cast<float>(color.a));
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}


void Renderer::draw() {
    clearScreen(glm::dvec4(0.1, 0.1, 0.1, 1.0));

    double aspectRatio = 1920.0 / 1080.0; // Adjust for your window size
    updateProjectionMatrix(aspectRatio);
    updateViewMatrix();

    drawGrid(10.0, 20, glm::dvec3(1.0, 1.0, 1.0));
}


void Renderer::drawGrid(double size, int divisions, const glm::dvec3& color) const {
    std::vector<double> vertices;
    double half = size / 2.0;
    double step = size / divisions;

    for (int i = 0; i <= divisions; ++i) {
        double offset = -half + step * i;

        vertices.push_back(-half); vertices.push_back(0.0); vertices.push_back(offset);
        vertices.push_back(half); vertices.push_back(0.0); vertices.push_back(offset);

        vertices.push_back(offset); vertices.push_back(0.0); vertices.push_back(-half);
        vertices.push_back(offset); vertices.push_back(0.0); vertices.push_back(half);
    }

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

    glUseProgram(m_GridShaderProgram);
    glUniform3dv(glGetUniformLocation(m_GridShaderProgram, "u_Color"), 1, glm::value_ptr(color));
    glBindVertexArray(VAO);

    std::size_t count = vertices.size() / 3;
    if (count > static_cast<std::size_t>(std::numeric_limits<GLsizei>::max())) {
        throw std::runtime_error("Too many vertices to draw; exceeds maximum GLsizei value.");
    }

    glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(count));

    glBindVertexArray(0);
    glDeleteBuffers(1, &VBO);
    glDeleteVertexArrays(1, &VAO);
}


void Renderer::handleMouseMovement(double xpos, double ypos) {
    if (!m_RightMouseHeld) return;

    if (m_FirstMouse) {
        m_LastMouseX = xpos;
        m_LastMouseY = ypos;
        m_FirstMouse = false;
    }

    double xOffset = xpos - m_LastMouseX;
    double yOffset = m_LastMouseY - ypos; // Inverted because y-coordinates go bottom to top
    m_LastMouseX = xpos;
    m_LastMouseY = ypos;

    const double sensitivity = 0.1; // Adjust sensitivity
    xOffset *= sensitivity;
    yOffset *= sensitivity;

    m_Yaw += xOffset;
    m_Pitch += yOffset;

    // Constrain pitch to prevent flipping
    if (m_Pitch > 89.0) m_Pitch = 89.0;
    if (m_Pitch < -89.0) m_Pitch = -89.0;

    // Update the front vector based on updated yaw and pitch
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
            m_FirstMouse = true; // Reset the first mouse movement
            glfwSetInputMode(m_Window, GLFW_CURSOR, GLFW_CURSOR_DISABLED); // Hide and lock cursor
        } else if (action == GLFW_RELEASE) {
            m_RightMouseHeld = false;
            glfwSetInputMode(m_Window, GLFW_CURSOR, GLFW_CURSOR_NORMAL); // Show and unlock cursor
        }
    }
}


void Renderer::handleKeyboardInput(GLFWwindow* window, double deltaTime) {
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
