//
// Created by devkon on 27/01/2025.
//

#ifndef DYNAMICSLAB_RENDERER_H
#define DYNAMICSLAB_RENDERER_H

#include "GLFW/glfw3.h"
#include "glm/glm.hpp"


class Renderer {
public:
    Renderer(GLFWwindow* window);
    ~Renderer();

    void clearScreen(const glm::vec4& color);
    void draw();

private:
    GLFWwindow* m_Window;
    GLuint m_GridShaderProgram;

    void initOpenGL();

    void drawGrid(float size, int divisions, const glm::vec3 &color);
};


#endif //DYNAMICSLAB_RENDERER_H
