#ifndef DYNAMICSLAB_RENDERER_H
#define DYNAMICSLAB_RENDERER_H

#include "glad/glad.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <memory>

#include "objects/Grid.h"
#include "objects/Cube.h"
#include "physics/RigidBody.h"


class Renderer {
public:
    Renderer();
    ~Renderer();

    bool initialize();
    void clear();
    void render();
    void shutdown();

    void addCube(std::unique_ptr<Cube> cube);

    void setProjectionMatrix(const glm::mat4& projection);
    void setViewMatrix(const glm::mat4& view);

private:
    glm::mat4 m_projectionMatrix;
    glm::mat4 m_viewMatrix;
    Grid      m_grid;

    std::vector<std::unique_ptr<Cube>> m_cubes;

};


#endif //DYNAMICSLAB_RENDERER_H
