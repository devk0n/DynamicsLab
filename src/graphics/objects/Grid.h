#ifndef GRID_H
#define GRID_H

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

class Grid {
public:
    Grid(int size = 5);
    ~Grid();

    bool initialize();
    void render(const glm::mat4& projection, const glm::mat4& view);
    void shutdown();

private:
    bool setupGrid();

    GLuint m_VAO;
    GLuint m_VBO;
    GLuint m_shaderProgram;
    int    m_size;
};

#endif // GRID_H
