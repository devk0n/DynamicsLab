#include "Grid.h"
#include <stdexcept>     // for std::runtime_error
#include <limits>        // for std::numeric_limits
#include <glm/gtc/type_ptr.hpp> // for value_ptr
#include <glm/gtc/matrix_transform.hpp>

Grid::Grid(double size, int divisions)
    : m_size(size)
    , m_divisions(divisions)
    , m_color(1.0, 1.0, 1.0) // default color is white
    , m_vao(0)
    , m_vbo(0)
    , m_vertexCount(0)
{
    createGrid();
}

Grid::~Grid() {
    glDeleteVertexArrays(1, &m_vao);
    glDeleteBuffers(1, &m_vbo);
}

void Grid::setColor(const glm::dvec3& color) {
    m_color = color;
}

void Grid::draw(GLuint shaderProgramID, const glm::dmat4& modelMatrix) {
    // Set uniform color
    GLint colorLoc = glGetUniformLocation(shaderProgramID, "u_Color");
    if (colorLoc != -1) {
        glUniform3dv(colorLoc, 1, glm::value_ptr(m_color));
    }

    // Set uniform model matrix
    GLint modelLoc = glGetUniformLocation(shaderProgramID, "u_Model");
    if (modelLoc != -1) {
        // Note: We must use the appropriate uniform function for double vs float
        // However, many drivers don't fully support double uniforms. In practice,
        // often you'd convert to float with "glm::mat4(modelMatrix)" and then call glUniformMatrix4fv.
        glm::mat4 modelFloat = glm::mat4(modelMatrix); // convert double matrix to float
        glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(modelFloat));
    }

    // Bind VAO and draw
    glBindVertexArray(m_vao);
    if (m_vertexCount > static_cast<std::size_t>(std::numeric_limits<GLsizei>::max())) {
        throw std::runtime_error("Too many vertices in Grid::draw()");
    }
    glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(m_vertexCount));
    glBindVertexArray(0);
}

void Grid::createGrid() {
    // Generate vertex data (double precision)
    std::vector<double> vertices;
    double half = m_size / 2.0;
    double step = m_size / static_cast<double>(m_divisions);

    for (int i = 0; i <= m_divisions; ++i) {
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

    m_vertexCount = vertices.size() / 3; // Each vertex is 3 components

    // Create VAO and VBO
    glGenVertexArrays(1, &m_vao);
    glGenBuffers(1, &m_vbo);

    glBindVertexArray(m_vao);
    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);

    std::size_t bufferSize = vertices.size() * sizeof(double);
    if (bufferSize > static_cast<std::size_t>(std::numeric_limits<GLsizeiptr>::max>()) {
        throw std::runtime_error("Grid buffer size exceeds allowable size.");
    }

    glBufferData(
        GL_ARRAY_BUFFER,
        static_cast<GLsizeiptr>(bufferSize),
        vertices.data(),
        GL_STATIC_DRAW
    );

    glVertexAttribPointer(
        0,                     // attribute location
        3,                     // component count (x,y,z)
        GL_DOUBLE,             // data type
        GL_FALSE,              // normalized
        3 * sizeof(double),    // stride
        nullptr
    );
    glEnableVertexAttribArray(0);

    // Unbind
    glBindVertexArray(0);
}