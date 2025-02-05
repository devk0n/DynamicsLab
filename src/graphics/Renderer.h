#ifndef DYNAMICSLAB_RENDERER_H
#define DYNAMICSLAB_RENDERER_H

#include <iostream>

#include "glm/glm.hpp"
#include "GLFW/glfw3.h"

#include "Camera.h"

class Renderer {
public:
    Renderer();
    ~Renderer();

    static void clearScreen();
    void draw();

private:
    GLuint m_gridShaderProgram;

    static GLuint compileShader(GLenum type, const char *source);
    static GLuint createShaderProgram(const char *vertexSource, const char *fragmentSource);

    static std::string loadShaderFromFile(const std::string &filepath);

    void drawGrid(double size, int divisions, const glm::dvec3& color) const;
};

#endif // DYNAMICSLAB_RENDERER_H
