#ifndef DYNAMICSLAB_CUBE_H
#define DYNAMICSLAB_CUBE_H

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glad/glad.h>
#include <string>
#include <iostream>

#include "utilities/ShaderUtils.h"


class Cube {
public:
    Cube(glm::vec3 position,
         glm::vec3 dimensions,
         glm::vec3 color,
         glm::quat orientation = glm::quat());
    ~Cube();

    void render(const glm::mat4& projection,
                const glm::mat4& view) const;
    void shutdown();

private:
    glm::vec3 m_position;
    glm::vec3 m_dimensions;
    glm::vec3 m_color;
    glm::quat m_orientation;

    GLuint m_shaderProgram;
    GLuint m_VAO, m_VBO;

    void setupCube();

    [[nodiscard]] glm::mat4 getModelMatrix() const;
};


#endif
