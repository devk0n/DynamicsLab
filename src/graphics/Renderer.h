#ifndef DYNAMICSLAB_RENDERER_H
#define DYNAMICSLAB_RENDERER_H

#include "glad/glad.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <string>
#include <vector>

#include "objects/Grid.h"

class Renderer {
public:
    Renderer();
    ~Renderer();

    bool initialize();
    void clear();
    void render();
    void shutdown();

    void setProjectionMatrix(const glm::mat4& projection);
    void setViewMatrix(const glm::mat4& view);

private:
    glm::mat4 m_projectionMatrix{};
    glm::mat4 m_viewMatrix;
    Grid      m_grid;

};


#endif //DYNAMICSLAB_RENDERER_H
