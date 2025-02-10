#ifndef RENDERER_H
#define RENDERER_H

#include <glad/glad.h>
#include <vector>
#include <glm/glm.hpp>
#include "physics/RigidBody.h"


class Renderer {
public:

    void initialize();
    void beginFrame();
    void render(const std::vector<RigidBody>& rigidBodies, const glm::mat4& view, const glm::mat4& projection);
    void endFrame();
    void shutdown();

    void setClearColor(float r, float g, float b, float a);

private:
    float m_clearColor[4] = {0.1f, 0.1f, 0.1f, 1.0f};

};


#endif // RENDERER_H
