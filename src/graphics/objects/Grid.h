//
// Created by devkon on 04/02/2025.
//

#ifndef DYNAMICSLAB_GRID_H
#define DYNAMICSLAB_GRID_H


#include <vector>
#include <glad/glad.h>
#include <glm/glm.hpp>

///
/// A class that creates and draws a grid on the XZ-plane.
/// Once constructed, the GPU resources remain loaded until the Grid is destroyed.
///
/// Usage:
///  1) Construct Grid object with desired size/divisions.
///  2) Call setColor(...) to change the grid color if needed.
///  3) Call draw(shaderProgramID, modelMatrix) each frame or whenever desired.
///
class Grid {
public:
    /// Construct a grid with the specified size and number of divisions.
    /// The grid is centered at the origin on the XZ-plane.
    Grid(double size, int divisions);

    /// Free GPU resources on destruction.
    ~Grid();

    /// Set the color of the grid (used in the fragment shader).
    void setColor(const glm::dvec3& color);

    /// Draw the grid using the specified shader program and model matrix.
    /// shaderProgramID: An already bound/used shader program.
    /// modelMatrix: The model transformation for the grid (identity = no transform).
    void draw(GLuint shaderProgramID, const glm::dmat4& modelMatrix);

private:
    double m_size;          ///< The total size of the grid.
    int    m_divisions;     ///< Number of subdivisions/lines.
    glm::dvec3 m_color;     ///< Grid color.

    GLuint m_vao;           ///< Vertex array object for the grid.
    GLuint m_vbo;           ///< Vertex buffer object for the grid.
    std::size_t m_vertexCount;

    // Helper to create/bind Vertex Array and Buffer
    void createGrid();
};



#endif //DYNAMICSLAB_GRID_H
